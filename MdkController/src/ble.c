/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* Atmel library includes. */
#include "asf.h"

#include <lib_aci.h>
#include <aci_setup.h>
#include "SEGGER_RTT.h"
#include "services.h"
#include "version.h"
#include "eep.h"
#include "sm.h"
#include "ble.h"
#include "mode_sms.h"
#include "mode_video.h"
#include "mode_pano.h"
#include "mode_astro.h"
#include "cam.h"
#include "io.h"
#include "utils.h"
#include "moco.h"

#define bleJOYSTICK_WATCHDOG_TIMER_RATE     (1000 / portTICK_RATE_MS)

static uint8_t joystick_wdg_enabled;
static uint8_t joystick_wdg_trigger;
static uint8_t mode = MOCO_MODE_SMS;

#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
    static services_pipe_type_mapping_t services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
    #define NUMBER_OF_PIPES 0
    static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif

static const hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] = SETUP_MESSAGES_CONTENT;
static aci_state_t aci_state;
static bool radio_ack_pending  = false;
static bool timing_change_done = false;

static void prvJoystickWatchdogTimerCallback(void *pvParameters);
static void prvAciEventHandlerTask(void *pvParameters);

void vBleInit(void)
{
    delay_ms(100);
    
    if (NULL != services_pipe_type_mapping)
    {
        aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
    }
    else
    {
        aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
    }
    aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
    aci_state.aci_setup_info.setup_msgs         = (hal_aci_data_t *)setup_msgs;
    aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;
    
    aci_state.aci_pins.board_name               = BOARD_DEFAULT;
    aci_state.aci_pins.reqn_pin                 = PIO_PC24_IDX;
    aci_state.aci_pins.rdyn_pin                 = PIO_PC23_IDX;
    aci_state.aci_pins.mosi_pin                 = SPI0_MOSI_GPIO;
    aci_state.aci_pins.miso_pin                 = SPI0_MISO_GPIO;
    aci_state.aci_pins.sck_pin                  = SPI0_SPCK_GPIO;
    aci_state.aci_pins.reset_pin                = PIO_PC22_IDX;
    aci_state.aci_pins.active_pin               = PIO_PC21_IDX;
    aci_state.aci_pins.optional_chip_sel_pin    = PIN_UNUSED;
    aci_state.aci_pins.interface_is_interrupt   = true;

    lib_aci_init(&aci_state, false);

    xTaskCreate(prvAciEventHandlerTask, "BleAciLoop", 600, NULL, 0, NULL);
    
    xTimerHandle xJoystickWatchdogTimer = xTimerCreate(
        (const char * const) "BleJoyWdgTmr",
        bleJOYSTICK_WATCHDOG_TIMER_RATE,
        pdTRUE,
        NULL,
        prvJoystickWatchdogTimerCallback);
    configASSERT(xJoystickWatchdogTimer);
    xTimerStart(xJoystickWatchdogTimer, 0); 
}

inline uint8_t ucBleGetMode(void)
{
	return mode;
}

inline void vBleSetMode(uint8_t value)
{
	mode = value;
}

inline void vBleJoystickTriggerReset(void)
{
	joystick_wdg_trigger = 0;
}

inline void vBleSetJoystickWatchdog(uint8_t enable)
{
	joystick_wdg_enabled = enable;
}

inline uint8_t ucBleGetJoystickWatchdog(void)
{
	return joystick_wdg_enabled;
}

bool bBleUpdateMoCoControlPointTx(uint8_t state, uint8_t * data, uint8_t length)
{
    uint8_t result[] = { 0, 0, 0, 0, 0, 0xff, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
    
    if (!lib_aci_is_pipe_available(&aci_state, PIPE_NMX_NMX_TX_TX))
    {
        return true;
    }

    if (aci_state.data_credit_available >= 1)
    {
        result[8] = state;
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

inline bool bBleUpdateMoCoControlPointTxOkData(uint8_t * data, uint8_t length)
{
    return bBleUpdateMoCoControlPointTx(1, data, length);
}

inline void vBleUpdateMoCoControlPointTxOk(void)
{
    bBleUpdateMoCoControlPointTx(1, NULL, 0);
}

inline void vBleUpdateMoCoControlPointTxError(void)
{
    bBleUpdateMoCoControlPointTx(0, NULL, 0);
}

bool bBleProcessMoCoControlPointRx(uint8_t subadr, uint8_t cmd, uint8_t * data, uint8_t data_length)
{
    SEGGER_RTT_printf(0, "MoCoBus Control RX: Adr %02x Cmd %02x\n", subadr, cmd);
        
    switch (subadr)
    {
    case 0x00:
        return bBleProcessMoCoControlPointRxMain(cmd, data, data_length);
    case 0x01:
    case 0x02:
    case 0x03:
        return bBleProcessMoCoControlPointRxMotor(subadr - 1, cmd, data, data_length);
    case 0x04:
        return bBleProcessMoCoControlPointRxCamera(cmd, data, data_length);
    }       
    
    return false;
}

void prvAciEventHandlerTask(void *pvParameters)
{
    static bool setup_required = false;
    hal_aci_evt_t aci_data;

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
                                    vTaskDelay(50 / portTICK_RATE_MS);
                                    
                                    char * name = eep_params.ble_device_name;
                                    lib_aci_set_local_data(&aci_state, PIPE_GAP_DEVICE_NAME_SET, (uint8_t *)name, strlen(name));
                                    char * version = versionFIRMWARE_VERSION_STRING;
                                    lib_aci_set_local_data(&aci_state, PIPE_DEVICE_INFORMATION_SOFTWARE_REVISION_STRING_SET, (uint8_t *)version, strlen(version));
                                    
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
                    
                    /* check if the peer has subscribed to the Heart Rate Measurement Characteristic for Notifications  */
                    if (!timing_change_done)
                    {
                        if (lib_aci_is_pipe_available(&aci_state, PIPE_NMX_NMX_RX_RX)
                            || lib_aci_is_pipe_available(&aci_state, PIPE_NMX_NMX_RX_RX_ACK_AUTO))
                        {
                            SEGGER_RTT_printf(0, "Request a change to the link timing\n");
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
                                bBleProcessMoCoControlPointRx(subadr, cmd, data, len);
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
                    
                    vTaskDelay(50 / portTICK_RATE_MS);
                    
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

static void prvJoystickWatchdogTimerCallback(void *pvParameters)
{
	static uint8_t last_joystick_wdg_trigger = 0;
	
	UNUSED(pvParameters);

	if (joystick_wdg_trigger && (last_joystick_wdg_trigger != joystick_wdg_trigger))
	{
		SEGGER_RTT_printf(0, "Joystick Watchdog Triggered! Stopping Motors...\n");
		for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
		{
			vSmStop(motor);
		}
	}
	
	last_joystick_wdg_trigger = joystick_wdg_trigger;
	joystick_wdg_trigger = 1;
}
