/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* Atmel library includes. */
#include "asf.h"

//#include <asf.h>
#include <lib_aci.h>
#include <aci_setup.h>
#include "SEGGER_RTT.h"
#include "services.h"
#include "eep.h"
#include "sm.h"

#define mainCHECK_TIMER_RATE                    (500 / portTICK_RATE_MS)
#define mainBLE_SLIDER_STATE_TX_TIMER_RATE      (200 / portTICK_RATE_MS)

#define mainDONT_BLOCK                          (0)

static void prvSetupHardware(void);
static void prvCheckTimerCallback(void *pvParameters);
static void prvBleSliderStateTxTimerCallback(void *pvParameters);

void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName);
void vApplicationTickHook(void);

void vTaskBleAciLoop(void *pvParameters);

sm_state_t sm_state;
bool slider_position_tx(int32_t position1, int32_t position2, int32_t position3, int32_t position4);

#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
	static services_pipe_type_mapping_t
		services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
	#define NUMBER_OF_PIPES 0
	static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif

static const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;

static aci_state_t aci_state;
static hal_aci_evt_t aci_data;
static hal_aci_data_t aci_cmd;

static bool radio_ack_pending  = false;
static bool timing_change_done = false;

/* Define how assert should function in the BLE library */
void __ble_assert(const char * file, uint16_t line)
{
	vAssertCalled(file, line);
}

bool slider_position_tx(int32_t position1, int32_t position2, int32_t position3, int32_t position4)
{
	bool status = false;

	if (lib_aci_is_pipe_available(&aci_state, PIPE_SLIDER_SLIDER_POSITION_TX) &&
		(aci_state.data_credit_available >= 1))
	{
		uint8_t buffer[16];
		*(int32_t *)&buffer[0] = position1;
		*(int32_t *)&buffer[4] = position2;
		*(int32_t *)&buffer[8] = position3;
		*(int32_t *)&buffer[12] = position4;
		
		status = lib_aci_send_data(PIPE_SLIDER_SLIDER_POSITION_TX, buffer, 16);
		if (status)
		{
			aci_state.data_credit_available--;
		}
	}
	return status;
}

bool slider_control_point_tx(uint8_t * buffer, uint8_t length)
{
	bool status = false;

	if (lib_aci_is_pipe_available(&aci_state, PIPE_SLIDER_SLIDER_CONTROL_POINT_TX) &&
	(aci_state.data_credit_available >= 1))
	{
		status = lib_aci_send_data(PIPE_SLIDER_SLIDER_CONTROL_POINT_TX, buffer, length);
		if (status)
		{
			aci_state.data_credit_available--;
		}
	}
	return status;
}

bool slider_process_control_point_rx(uint8_t * buffer, uint8_t length)
{
	bool status = false;

	if (lib_aci_is_pipe_available(&aci_state, PIPE_SLIDER_SLIDER_CONTROL_POINT_RX_ACK_AUTO))
	{
		SEGGER_RTT_printf(0, "Slider Control RX: Cmd %02x\n", *buffer);
		switch(*buffer)
		{
			case 0x90:
				SEGGER_RTT_printf(0, "Slider Control RX: Stop\n");
				sm_stop();
			
				uint8_t result = 0;
				slider_control_point_tx(&result, 1);
			
				status = true;
				break;
			case 0x91:
				{
					SEGGER_RTT_printf(0, "Slider Control RX: Move Left\n");
					sm_start(SM_CCW);
				
					uint8_t result = 0;
					slider_control_point_tx(&result, 1);
				
					status = true;
				}
				break;
			case 0x92:
				{
					SEGGER_RTT_printf(0, "Slider Control RX: Move Right\n");
					sm_start(SM_CW);
				
					uint8_t result = 0;
					slider_control_point_tx(&result, 1);
				
					status = true;
				}
				break;
			case 0x93:
				{
					int32_t steps = *(int32_t *)(buffer+1);
					SEGGER_RTT_printf(0, "Slider Control RX: Move %d\n", steps);
					sm_move(steps);
					
					uint8_t result = 0;
					slider_control_point_tx(&result, 1);
					
					status = true;
				}
				break;
			case 0x99:
				{
					int32_t position = *(int32_t *)(buffer+1);
					SEGGER_RTT_printf(0, "Slider Control RX: Goto %d\n", position);
					int32_t steps = position - sm_get_position();
					sm_move(steps);
				
					uint8_t result = 0;
					slider_control_point_tx(&result, 1);
				
					status = true;
				}
				break;
		}
	}
	return status;
}

int main(void)
{
	SEGGER_RTT_printf(0, "Startup...\n");
	
	prvSetupHardware();

	xTimerHandle xCheckTimer = xTimerCreate(
		(const char * const) "ChkTmr",
		mainCHECK_TIMER_RATE,
		pdTRUE,
		NULL,
		prvCheckTimerCallback);

	/* Sanity check the timer's creation, then start the timer.  The timer
	will not actually start until the FreeRTOS kernel is started. */
	configASSERT(xCheckTimer);
	xTimerStart(xCheckTimer, mainDONT_BLOCK);

	xTimerHandle xBleSliderStateTxTimer = xTimerCreate(
		(const char * const) "BleStTxTmr",
		mainBLE_SLIDER_STATE_TX_TIMER_RATE,
		pdTRUE,
		NULL,
		prvBleSliderStateTxTimerCallback);
	configASSERT(xBleSliderStateTxTimer);
	xTimerStart(xBleSliderStateTxTimer, mainDONT_BLOCK);
	
	xTaskCreate(vTaskBleAciLoop, "BleAciLoop", 300, NULL, 0, NULL);

	/* Start the RTOS scheduler. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	for (;;) {}
}

static void prvCheckTimerCallback(void *pvParameters)
{
	UNUSED(pvParameters);
	
	ioport_toggle_pin_level(LED0_GPIO);
}

static void prvBleSliderStateTxTimerCallback(void *pvParameters)
{
	UNUSED(pvParameters);
	
	slider_position_tx(sm_get_position(), 0, 0, 0);
}

/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
	/* ASF function to setup clocking. */
	sysclk_init();

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_SetPriorityGrouping(0);

	/* Atmel library function to setup for the evaluation kit being used. */
	board_init();

	eep_init();
	sm_init(&sm_state);
	
	delay_ms(100);
	
	if (NULL != services_pipe_type_mapping)
	{
		aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
	}
	else
	{
		aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
	}
	aci_state.aci_setup_info.number_of_pipes	= NUMBER_OF_PIPES;
	aci_state.aci_setup_info.setup_msgs			= (hal_aci_data_t *)setup_msgs;
	aci_state.aci_setup_info.num_setup_msgs		= NB_SETUP_MESSAGES;
	
	aci_state.aci_pins.board_name				= BOARD_DEFAULT;
	aci_state.aci_pins.reqn_pin					= PIO_PC21_IDX;
	aci_state.aci_pins.rdyn_pin					= PIO_PC22_IDX;
	aci_state.aci_pins.mosi_pin					= SPI0_MOSI_GPIO;
	aci_state.aci_pins.miso_pin					= SPI0_MISO_GPIO;
	aci_state.aci_pins.sck_pin					= SPI0_SPCK_GPIO;
	aci_state.aci_pins.reset_pin				= PIO_PC23_IDX;
	aci_state.aci_pins.active_pin				= PIN_UNUSED;
	aci_state.aci_pins.optional_chip_sel_pin	= PIN_UNUSED;
	aci_state.aci_pins.interface_is_interrupt	= true;

	lib_aci_init(&aci_state, true);
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	
	SEGGER_RTT_printf(0, "ERROR: vApplicationMallocFailedHook\n");
	
	for (;;) {
	}
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	function, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
	
	//vTaskBleAciLoop(NULL);
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName)
{
	(void) pcTaskName;
	(void) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	
	SEGGER_RTT_printf(0, "ERROR: vApplicationStackOverflowHook (%s)\n", pcTaskName);
	
	for (;;) {}
}

/*-----------------------------------------------------------*/

void vApplicationTickHook(void)
{
	/* This function will be called by each tick interrupt if
	configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
	added here, but the tick hook is called from an interrupt context, so
	code must not attempt to block, and only the interrupt safe FreeRTOS API
	functions can be used (those that end in FromISR()). */
}

/*-----------------------------------------------------------*/

void vAssertCalled(const char *file, uint32_t line)
{
	volatile uint32_t block_var = 0, line_in;
	const char *file_in;

	/* These assignments are made to prevent the compiler optimizing the
	values away. */
	file_in = file;
	line_in = line;
	(void) file_in;
	(void) line_in;
	
	SEGGER_RTT_printf(0, "ERROR %s: %d\n", file, line);	

	taskENTER_CRITICAL();
	{
		while (block_var == 0) {
			/* Set block_var to a non-zero value in the debugger to
			step out of this function. */
		}
	}
	taskEXIT_CRITICAL();
}

void vTaskBleAciLoop(void *pvParameters)
{
	static bool setup_required = false;

	for (;;)
	{
		// We enter the if statement only when there is a ACI event available to be processed
		if (lib_aci_event_get(&aci_state, &aci_data))
		{
			aci_evt_t * aci_evt;
			aci_evt = &aci_data.evt;

			switch(aci_evt->evt_opcode)
			{
				case ACI_EVT_DEVICE_STARTED:
					{
						aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
						switch (aci_evt->params.device_started.device_mode)
						{
							case ACI_DEVICE_SETUP:
								/**
								When the device is in the setup mode
								*/
								aci_state.device_state = ACI_DEVICE_SETUP;
								SEGGER_RTT_printf(0, "Evt Device Started: Setup\n");
								setup_required = true;
								break;

							case ACI_DEVICE_STANDBY:
								aci_state.device_state = ACI_DEVICE_STANDBY;
								SEGGER_RTT_printf(0, "Evt Device Started: Standby\n");
								if (aci_evt->params.device_started.hw_error)
								{
									vTaskDelay(20 / portTICK_RATE_MS); //Magic number used to make sure the HW error event is handled correctly.
								}
								else
								{
									lib_aci_connect(300, 320);
									SEGGER_RTT_printf(0, "Advertising started\n");
								}
								break;
								
							default:
								break;
						}
					}
					break; //ACI Device Started Event

				case ACI_EVT_CMD_RSP:
					//If an ACI command response event comes with an error -> stop
					if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status )
					{
						//ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
						//TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
						//all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command

						SEGGER_RTT_printf(0, "ACI Status of ACI Evt Cmd Rsp 0x%02x\n", aci_evt->params.cmd_rsp.cmd_status);
						SEGGER_RTT_printf(0, "ACI Command 0x%02x\n", aci_evt->params.cmd_rsp.cmd_opcode);
						SEGGER_RTT_printf(0, "Evt Cmd respone: Error.\n");
						while (1);
					}
					break;

				case ACI_EVT_PIPE_STATUS:
					SEGGER_RTT_printf(0, "Evt Pipe Status\n");
					
					/* check if the peer has subscribed to the Heart Rate Measurement Characteristic for Notifications	*/
					if (lib_aci_is_pipe_available(&aci_state, PIPE_SLIDER_SLIDER_CONTROL_POINT_TX) && (false == timing_change_done))
					{
						/*
						Request a change to the link timing as set in the GAP -> Preferred Peripheral Connection Parameters
						Change the setting in nRFgo studio -> nRF8001 configuration -> GAP Settings and recompile the xml file.
						*/
						lib_aci_change_timing_GAP_PPCP();
						timing_change_done = true;
					}
					break;

				case ACI_EVT_TIMING:
					/*
					Link timing has changed.
					*/
					SEGGER_RTT_printf(0, "Timing changed: %02x\n", aci_evt->params.timing.conn_rf_interval);
					break;

				case ACI_EVT_CONNECTED:
					radio_ack_pending  = false;
					timing_change_done = false;
					aci_state.data_credit_available = aci_state.data_credit_total;
					SEGGER_RTT_printf(0, "Evt Connected\n");
					break;

				case ACI_EVT_DATA_CREDIT:
					aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
					/**
					Bluetooth Radio ack received from the peer radio for the data packet sent.
					This also signals that the buffer used by the nRF8001 for the data packet is available again.
					*/
					radio_ack_pending = false;
					break;

				case ACI_EVT_PIPE_ERROR:
					/**
					Send data failed. ACI_EVT_DATA_CREDIT will not come.
					This can happen if the pipe becomes unavailable by the peer unsubscribing to the Heart Rate
					Measurement characteristic.
					This can also happen when the link is disconnected after the data packet has been sent.
					*/
					radio_ack_pending = false;

					//See the appendix in the nRF8001 Product Specication for details on the error codes
					SEGGER_RTT_printf(0, "ACI Evt Pipe Error: Pipe #:%d Pipe Error Code: 0x%02x\n", aci_evt->params.pipe_error.pipe_number, aci_evt->params.pipe_error.error_code);

					//Increment the credit available as the data packet was not sent.
					//The pipe error also represents the Attribute protocol Error Response sent from the peer and that should not be counted
					//for the credit.
					if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code)
					{
						aci_state.data_credit_available++;
					}
					break;

				case ACI_EVT_DISCONNECTED:
					/**
					Advertise again if the advertising timed out.
					*/
					if (ACI_STATUS_ERROR_ADVT_TIMEOUT == aci_evt->params.disconnected.aci_status)
					{
						SEGGER_RTT_printf(0, "Evt Disconnected -> Advertising timed out\n");
						/*SEGGER_RTT_printf(0, "nRF8001 going to sleep\n");
						lib_aci_sleep();
						aci_state.device_state = ACI_DEVICE_SLEEP;*/
						lib_aci_connect(300, 320);
						SEGGER_RTT_printf(0, "Advertising started\n");
					}
					if (ACI_STATUS_EXTENDED == aci_evt->params.disconnected.aci_status)
					{
						SEGGER_RTT_printf(0, "Evt Disconnected -> Link lost. Bluetooth Error code = 0x%02x\n", aci_evt->params.disconnected.btle_status);
						lib_aci_connect(300, 320);
						SEGGER_RTT_printf(0, "Advertising started\n");
					}
					break;

				case ACI_EVT_DATA_RECEIVED:
					SEGGER_RTT_printf(0, "Pipe RX: No %d\n", aci_evt->params.data_received.rx_data.pipe_number);
					if (PIPE_SLIDER_SLIDER_CONTROL_POINT_RX_ACK_AUTO == aci_evt->params.data_received.rx_data.pipe_number)
					{
						slider_process_control_point_rx(&aci_evt->params.data_received.rx_data.aci_data[0], aci_evt->len - 2);
					}
					break;

				case ACI_EVT_HW_ERROR:
					SEGGER_RTT_printf(0, "HW error: %d\n", aci_evt->params.hw_error.line_num);
					for (uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
					{
						SEGGER_RTT_printf(0, (const char *)&aci_evt->params.hw_error.file_name[counter]);
					}
					SEGGER_RTT_printf(0, "\n");
					
					lib_aci_connect(300, 320);
					SEGGER_RTT_printf(0, "Advertising started\n");
					break;
					
				default:
					break;
			}
		}
		else
		{
			// If No event in the ACI Event queue and No event in the ACI Command queue
			//vTaskDelay(10 / portTICK_RATE_MS);
		}
	
		if (setup_required)
		{
			if (SETUP_SUCCESS == do_aci_setup(&aci_state))
			{
				setup_required = false;
			}
		}
	}
}