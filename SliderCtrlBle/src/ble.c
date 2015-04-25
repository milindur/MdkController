/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* Atmel library includes. */
#include "asf.h"

#include "trcUser.h"
#include "trcConfig.h"
#include "trcHardwarePort.h"

#include <lib_aci.h>
#include <aci_setup.h>
#include "SEGGER_RTT.h"
#include "services.h"
#include "eep.h"
#include "sm.h"
#include "ble.h"
#include "slider.h"
#include "utils.h"

#define MOCO_VALUE_BYTE		0
#define MOCO_VALUE_UINT		1
#define MOCO_VALUE_INT		2
#define MOCO_VALUE_LONG		3
#define MOCO_VALUE_ULONG	4
#define MOCO_VALUE_FLOAT	5
#define MOCO_VALUE_STRING	6

#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
	static services_pipe_type_mapping_t services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
	#define NUMBER_OF_PIPES 0
	static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif

static const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] = SETUP_MESSAGES_CONTENT;

static aci_state_t aci_state;
static hal_aci_evt_t aci_data;
static hal_aci_data_t aci_cmd;

static bool radio_ack_pending  = false;
static bool timing_change_done = false;
static traceLabel mot_user_event_channel;

static bool prbBleUpdateSliderControlPointTx(uint8_t * buffer, uint8_t length);
static bool prbBleProcessSliderControlPointRx(uint8_t subadr, uint8_t cmd, uint8_t * data, uint8_t data_length);
static bool prbBleProcessSliderControlPointRxMain(uint8_t cmd, uint8_t * data, uint8_t data_length);
static bool prbBleProcessSliderControlPointRxMotor(uint8_t motor, uint8_t cmd, uint8_t * data, uint8_t data_length);
static bool prbBleProcessSliderControlPointRxCamera(uint8_t cmd, uint8_t * data, uint8_t data_length);
static void prvBleUpdateSliderControlPointTxOk(void);
static void prvAciEventHandlerTask(void *pvParameters);

void vBleInit(void)
{
	mot_user_event_channel = xTraceOpenLabel("MotCont");
	
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
	aci_state.aci_pins.reqn_pin					= PIO_PC24_IDX;
	aci_state.aci_pins.rdyn_pin					= PIO_PC23_IDX;
	aci_state.aci_pins.mosi_pin					= SPI0_MOSI_GPIO;
	aci_state.aci_pins.miso_pin					= SPI0_MISO_GPIO;
	aci_state.aci_pins.sck_pin					= SPI0_SPCK_GPIO;
	aci_state.aci_pins.reset_pin				= PIO_PC22_IDX;
	aci_state.aci_pins.active_pin				= PIO_PC21_IDX;
	aci_state.aci_pins.optional_chip_sel_pin	= PIN_UNUSED;
	aci_state.aci_pins.interface_is_interrupt	= true;

	lib_aci_init(&aci_state, true);

	xTaskCreate(prvAciEventHandlerTask, "BleAciLoop", 600, NULL, 0, NULL);
}

bool prbBleUpdateSliderControlPointTx(uint8_t * data, uint8_t length)
{
	uint8_t result[] = { 0, 0, 0, 0, 0, 0xff, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	
	if (!lib_aci_is_pipe_available(&aci_state, PIPE_NMX_NMX_TX_TX))
	{
		return true;
	}

	if (aci_state.data_credit_available >= 1)
	{
		result[9] = length;
		memcpy(&result[10], data, length);
		
		if (lib_aci_send_data(PIPE_NMX_NMX_TX_TX, result, length + 10))
		{
			aci_state.data_credit_available--;
			return true;
		}
	}
	
	return false;
}

static void prvBleUpdateSliderControlPointTxOk(void)
{
	prbBleUpdateSliderControlPointTx(NULL, 0);
}

bool prbBleProcessSliderControlPointRx(uint8_t subadr, uint8_t cmd, uint8_t * data, uint8_t data_length)
{
	SEGGER_RTT_printf(0, "Slider Control RX: Adr %02x Cmd %02x\n", subadr, cmd);
		
	switch (subadr)
	{
	case 0x00:
		return prbBleProcessSliderControlPointRxMain(cmd, data, data_length);
	case 0x01:
	case 0x02:
	case 0x03:
		return prbBleProcessSliderControlPointRxMotor(subadr - 1, cmd, data, data_length);
	case 0x04:
		return prbBleProcessSliderControlPointRxCamera(cmd, data, data_length);
	}		
	
	return false;
}

bool prbBleProcessSliderControlPointRxMain(uint8_t cmd, uint8_t * data, uint8_t data_length)
{
	switch (cmd)
	{
	case 0x02:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Start Program\n");
			vSliderStart();
						
			prvBleUpdateSliderControlPointTxOk();
			return true;
		}
	case 0x04:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Stop Program\n");
			vSliderStop();
						
			prvBleUpdateSliderControlPointTxOk();
			return true;
		}
	case 0x0A:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Move Home\n");
			ucSmMove(0, -lSmGetPosition(0));
			ucSmMove(1, -lSmGetPosition(1));
			ucSmMove(2, -lSmGetPosition(2));
						
			prvBleUpdateSliderControlPointTxOk();
			return true;
		}
	case 0x19:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Move Start Point\n");
			int32_t steps = eep_params.slider_positions[0].pos[0] - lSmGetPosition(0);
			ucSmMove(0, steps);
			steps = eep_params.slider_positions[0].pos[1] - lSmGetPosition(1);
			ucSmMove(1, steps);
			steps = eep_params.slider_positions[0].pos[2] - lSmGetPosition(2);
			ucSmMove(2, steps);
						
			prvBleUpdateSliderControlPointTxOk();
			return true;
		}
	case 0x1A:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Set Program Start Point\n");
			eep_params.slider_positions[0].pos[0] = lSmGetPosition(0);
			eep_params.slider_positions[0].pos[1] = lSmGetPosition(1);
			eep_params.slider_positions[0].pos[2] = lSmGetPosition(2);
			
			prvBleUpdateSliderControlPointTxOk();
			return true;
		}
	case 0x1B:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Set Program End Point\n");
			eep_params.slider_positions[1].pos[0] = lSmGetPosition(0);
			eep_params.slider_positions[1].pos[1] = lSmGetPosition(1);
			eep_params.slider_positions[1].pos[2] = lSmGetPosition(2);
						
			prvBleUpdateSliderControlPointTxOk();
			return true;
		}
	case 0x1D:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Swap Program Start/End Points\n");
			
			vUtilsSwap(&eep_params.slider_positions[0].pos[0], &eep_params.slider_positions[1].pos[0]);
			vUtilsSwap(&eep_params.slider_positions[0].pos[1], &eep_params.slider_positions[1].pos[1]);
			vUtilsSwap(&eep_params.slider_positions[0].pos[2], &eep_params.slider_positions[1].pos[2]);
						
			prvBleUpdateSliderControlPointTxOk();
			return true;
		}
	case 0x64:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Get Firmware Version\n");
		
			uint8_t result[] = { MOCO_VALUE_LONG, 0, 0, 0, 0 };
			*(uint32_t *)&result[1] = __builtin_bswap32(27);
			prbBleUpdateSliderControlPointTx(result, 5);
			return true;
		}
	case 0x65:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Get Run Status\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			if (ucSliderGetState() != SLIDER_STATE_STOP)
			{
				result[1] = 2;
			}
			prbBleUpdateSliderControlPointTx(result, 2);
			return true;
		}
	case 0x66:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Get Run Time\n");
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			*(uint32_t *)&result[1] = __builtin_bswap32(ulSliderGetCurrentTime());
			prbBleUpdateSliderControlPointTx(result, 5);
			return true;
		}
	case 0x76:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Get SMS / Continuous Program Mode\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			prbBleUpdateSliderControlPointTx(result, 2);
			return true;
		}
	case 0x7B:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Get Program % Complete\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			result[1] = ucSliderGetProgress();
			prbBleUpdateSliderControlPointTx(result, 2);
			return true;
		}
	case 0x7D:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Get Total Program Run Time\n");
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			*(uint32_t *)&result[1] = __builtin_bswap32(ulSliderGetOverallTime());
			prbBleUpdateSliderControlPointTx(result, 5);
			return true;
		}
	case 0x7E:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Get Total Program Run Time\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			if (ucSliderGetState() == SLIDER_STATE_STOP)
			{
				result[1] = 1;
			}
			prbBleUpdateSliderControlPointTx(result, 2);
			return true;
		}
	}

	prvBleUpdateSliderControlPointTxOk();
	return false;
}

bool prbBleProcessSliderControlPointRxMotor(uint8_t motor, uint8_t cmd, uint8_t * data, uint8_t data_length)
{
	switch (cmd)
	{
	case 0x04:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: [MOTOR%d] Stop\n", motor);
			vSmStop(motor);
			
			prvBleUpdateSliderControlPointTxOk();
			return true;
		}
	case 0x0D:
		{
			uint32_t tmp = __builtin_bswap32(*(uint32_t *)data);
			float speed = *(float *)&tmp;

			char buffer[20];
			sprintf(buffer, "%.3f", speed);
			SEGGER_RTT_printf(0, "Slider Control RX: [MOTOR%d] Move Continuous %s\n", motor, buffer);
			
			bSmMoveContinuous(motor, (int32_t) speed);
		
			prvBleUpdateSliderControlPointTxOk();
			return true;
		}
	case 0x6B:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: [MOTOR%d] Get Motor Running, res=%d\n", motor, ucSmGetState(motor));
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			result[1] = ucSmGetState(motor) != SM_STATE_STOP ? 1 : 0;
			prbBleUpdateSliderControlPointTx(result, 2);
			return true;
		}
	}

	prvBleUpdateSliderControlPointTxOk();
	return false;
}

bool prbBleProcessSliderControlPointRxCamera(uint8_t cmd, uint8_t * data, uint8_t data_length)
{
	switch (cmd)
	{
	case 0x06:
		{
			uint16_t count = __builtin_bswap16(*(uint16_t *)data);
			SEGGER_RTT_printf(0, "Slider Control RX: Max Shots %d\n", count);
			eep_params.slider_count = count;
						
			prvBleUpdateSliderControlPointTxOk();
			return true;
		}
	case 0x0A:
		{
			uint32_t interval = __builtin_bswap32(*(uint32_t *)data);
			SEGGER_RTT_printf(0, "Slider Control RX: Interval %d\n", interval);
			eep_params.slider_interval = interval;
						
			prvBleUpdateSliderControlPointTxOk();
			return true;
		}
	case 0x6C:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Get Interval\n");
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			*(uint32_t *)&result[1] = __builtin_bswap32(eep_params.slider_interval);
			prbBleUpdateSliderControlPointTx(result, 5);
			return true;
		}
	case 0x6D:
		{
			SEGGER_RTT_printf(0, "Slider Control RX: Get Current Shots\n");
			uint8_t result[] = { MOCO_VALUE_UINT, 0, 0 };
			*(uint16_t *)&result[1] = __builtin_bswap16((uint16_t)ulSliderGetCurrentCycle());
			prbBleUpdateSliderControlPointTx(result, 3);
			return true;
		}
	}
	
	prvBleUpdateSliderControlPointTxOk();
	return false;
}

void prvAciEventHandlerTask(void *pvParameters)
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
					if (!timing_change_done)
					{
						if (lib_aci_is_pipe_available(&aci_state, PIPE_NMX_NMX_RX_RX)
							|| lib_aci_is_pipe_available(&aci_state, PIPE_NMX_NMX_RX_RX_ACK_AUTO))
						{
							/*
							Request a change to the link timing as set in the GAP -> Preferred Peripheral Connection Parameters
							Change the setting in nRFgo studio -> nRF8001 configuration -> GAP Settings and recompile the xml file.
							*/
							lib_aci_change_timing_GAP_PPCP();
							timing_change_done = true;
						}
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
					if ((PIPE_NMX_NMX_RX_RX == aci_evt->params.data_received.rx_data.pipe_number)
						|| (PIPE_NMX_NMX_RX_RX_ACK_AUTO == aci_evt->params.data_received.rx_data.pipe_number))
					{
						uint8_t msglen = aci_evt->len - 2;
						if (msglen >= 10)
						{
							uint8_t * msg = &aci_evt->params.data_received.rx_data.aci_data[0];
							
							uint32_t * header1 = (uint32_t *)&msg[0];
							uint16_t * header2 = (uint16_t *)&msg[4];
							uint8_t adr = msg[6];
							uint8_t subadr = msg[7];
							uint8_t cmd = msg[8];
							uint8_t len = msg[9];
							uint8_t * data = &msg[10];
							
							if (*header1 == 0 && *header2 == 0xff00 && adr == 3)
							{
								prbBleProcessSliderControlPointRx(subadr, cmd, data, len);
							}
						}
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
			vTaskDelay(5 / portTICK_RATE_MS);
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
