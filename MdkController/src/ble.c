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

#define MOCO_VALUE_BYTE		0
#define MOCO_VALUE_UINT		1
#define MOCO_VALUE_INT		2
#define MOCO_VALUE_LONG		3
#define MOCO_VALUE_ULONG	4
#define MOCO_VALUE_FLOAT	5
#define MOCO_VALUE_STRING	6

#define MOCO_MODE_SMS			0
#define MOCO_MODE_TL_CONT		1
#define MOCO_MODE_VIDEO_CONT	2
#define MOCO_MODE_PANO			100
#define MOCO_MODE_ASTRO			101

#define bleJOYSTICK_WATCHDOG_TIMER_RATE		(1000 / portTICK_RATE_MS)

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

static bool prbBleUpdateMoCoControlPointTx(uint8_t state, uint8_t * buffer, uint8_t length);
static bool prbBleProcessMoCoControlPointRx(uint8_t subadr, uint8_t cmd, uint8_t * data, uint8_t data_length);
static bool prbBleProcessMoCoControlPointRxMain(uint8_t cmd, uint8_t * data, uint8_t data_length);
static bool prbBleProcessMoCoControlPointRxMotor(uint8_t motor, uint8_t cmd, uint8_t * data, uint8_t data_length);
static bool prbBleProcessMoCoControlPointRxCamera(uint8_t cmd, uint8_t * data, uint8_t data_length);
static bool prbBleUpdateMoCoControlPointTxOkData(uint8_t * data, uint8_t length);
static void prvBleUpdateMoCoControlPointTxOk(void);
static void prvBleUpdateMoCoControlPointTxError(void);
static void prvAciEventHandlerTask(void *pvParameters);
static void prvJoystickWatchdogTimerCallback(void *pvParameters);

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
	
	xTimerHandle xJoystickWatchdogTimer = xTimerCreate(
		(const char * const) "BleJoyWdgTmr",
		bleJOYSTICK_WATCHDOG_TIMER_RATE,
		pdTRUE,
		NULL,
		prvJoystickWatchdogTimerCallback);
	configASSERT(xJoystickWatchdogTimer);
	xTimerStart(xJoystickWatchdogTimer, 0);	
}

bool prbBleUpdateMoCoControlPointTx(uint8_t state, uint8_t * data, uint8_t length)
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

bool prbBleUpdateMoCoControlPointTxOkData(uint8_t * data, uint8_t length)
{
	return prbBleUpdateMoCoControlPointTx(1, data, length);
}

void prvBleUpdateMoCoControlPointTxOk(void)
{
	prbBleUpdateMoCoControlPointTx(1, NULL, 0);
}

void prvBleUpdateMoCoControlPointTxError(void)
{
	prbBleUpdateMoCoControlPointTx(0, NULL, 0);
}

bool prbBleProcessMoCoControlPointRx(uint8_t subadr, uint8_t cmd, uint8_t * data, uint8_t data_length)
{
	SEGGER_RTT_printf(0, "MoCoBus Control RX: Adr %02x Cmd %02x\n", subadr, cmd);
		
	switch (subadr)
	{
	case 0x00:
		return prbBleProcessMoCoControlPointRxMain(cmd, data, data_length);
	case 0x01:
	case 0x02:
	case 0x03:
		return prbBleProcessMoCoControlPointRxMotor(subadr - 1, cmd, data, data_length);
	case 0x04:
		return prbBleProcessMoCoControlPointRxCamera(cmd, data, data_length);
	}		
	
	return false;
}

bool prbBleProcessMoCoControlPointRxMain(uint8_t cmd, uint8_t * data, uint8_t data_length)
{
	switch (cmd)
	{
	case 0x02:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Start Program\n");
			if (mode == MOCO_MODE_SMS)
			{
				if (bModeSmsIsPaused())
				{
					vModeSmsResume();
				}
				else
				{
					vModeSmsStart();
				}
			}
			else if (mode == MOCO_MODE_VIDEO_CONT)
			{
				if (bModeVideoIsPaused())
				{
					vModeVideoResume();
				}
				else
				{
					vModeVideoStart();
				}
			}
			else if (mode == MOCO_MODE_PANO)
			{
				if (bModePanoIsPaused())
				{
					vModePanoResume();
				}
				else
				{
					vModePanoStart();
				}
			}
			else if (mode == MOCO_MODE_ASTRO)
			{
				if (bModeAstroIsPaused())
				{
					vModeAstroResume();
				}
				else
				{
                    if (data_length == 1)
                    {
                        uint8_t dir = data[0] == 0 ? MODE_ASTRO_DIR_NORTH : MODE_ASTRO_DIR_SOUTH;
                        vModeAstroStart(dir);
                    }
                    else
                    {
                        vModeAstroStart(MODE_ASTRO_DIR_NORTH);
                    }
				}
			}
									
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x03:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Pause Program\n");
			if (mode == MOCO_MODE_SMS)
			{
				vModeSmsPause();
			}
			else if (mode == MOCO_MODE_VIDEO_CONT)
			{
				vModeVideoPause();
			}
			else if (mode == MOCO_MODE_PANO)
			{
				vModePanoPause();
			}
			else if (mode == MOCO_MODE_ASTRO)
			{
			    vModeAstroPause();
			}
		
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x04:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Stop Program\n");
			if (mode == MOCO_MODE_SMS)
			{
				vModeSmsStop();
			}
			else if (mode == MOCO_MODE_VIDEO_CONT)
			{
				vModeVideoStop();
			}
			else if (mode == MOCO_MODE_PANO)
			{
				vModePanoStop();
			}
			else if (mode == MOCO_MODE_ASTRO)
			{
				vModeAstroStop();
			}
						
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x0A:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Move Home\n");
			for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
			{
				if (eep_params.sm[motor].power_save == 0) vSmEnable(motor, 1);
				if (eep_params.sm[motor].power_save == 2) vSmEnable(motor, 2);
				ucSmMove(motor, -lSmGetPosition(motor));
			}
						
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x0E:
		{
			uint8_t enable = data[0];
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Set Joystick Watchdog %d\n", enable);
			joystick_wdg_enabled = enable;
			joystick_wdg_trigger = 0;
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x16:
		{
			if (!(bModeSmsIsPaused() || bModeVideoIsPaused() || bModePanoIsPaused() || bModeAstroIsPaused()
                || bModeSmsIsRunning() || bModeVideoIsRunning() || bModePanoIsRunning() || bModeAstroIsRunning()))
			{
			    mode = data[0];
			}
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Set Program Mode %d\n", mode);
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x18:
		{
			uint8_t ping_pong = data[0];
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Set Ping-Pong Flag %d\n", ping_pong);
			eep_params.mode_video_ping_pong = ping_pong;
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x19:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Move Start Point\n");
			for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
			{
				if (eep_params.sm[motor].power_save == 0) vSmEnable(motor, 1);
				if (eep_params.sm[motor].power_save == 2) vSmEnable(motor, 2);
				int32_t steps = 0;
				if (mode == MOCO_MODE_SMS || mode == MOCO_MODE_VIDEO_CONT)
				{
					steps = eep_params.mode_sms_positions[0].pos[motor] - lSmGetPosition(motor);
				}
				else if (mode == MOCO_MODE_PANO)
				{
					steps = eep_params.mode_pano_position_start.pos[motor] - lSmGetPosition(motor);
				}
				ucSmMove(motor, steps);
			}
						
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x1A:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Set Program Start Point\n");
			for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
			{
				eep_params.mode_sms_positions[0].pos[motor] = lSmGetPosition(motor);
				eep_params.mode_pano_position_stop.pos[motor] = lSmGetPosition(motor);
				SEGGER_RTT_printf(0, "MoCoBus Control RX: Set Program Start Point %d: %d\n", motor, lSmGetPosition(motor));
			}
			
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x1B:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Set Program End Point\n");
			for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
			{
				eep_params.mode_sms_positions[1].pos[motor] = lSmGetPosition(motor);
				eep_params.mode_pano_position_start.pos[motor] = lSmGetPosition(motor);
				SEGGER_RTT_printf(0, "MoCoBus Control RX: Set Program End Point %d: %d\n", motor, lSmGetPosition(motor));
			}
						
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x1D:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Swap Program Start/End Points\n");
			
			if (mode == MOCO_MODE_SMS || mode == MOCO_MODE_VIDEO_CONT)
			{
				for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
				{
					vUtilsSwap(&eep_params.mode_sms_positions[0].pos[motor], &eep_params.mode_sms_positions[1].pos[motor]);
				}
			}
						
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x32:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Set Graffik Mode\n");
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x33:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Set App Mode\n");
			return true;
		}
	case 0x64:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Firmware Version\n");
			uint32_t tmp = __builtin_bswap32(versionFIRMWARE_VERSION);
			uint8_t result[] = { MOCO_VALUE_LONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			prbBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
	case 0x65:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Run Status\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			if (bModeSmsIsPaused() || bModeVideoIsPaused() || bModePanoIsPaused() || bModeAstroIsPaused())
			{
    			result[1] = 1;
			}
			else if (bModeSmsIsRunning() || bModeVideoIsRunning() || bModePanoIsRunning() || bModeAstroIsRunning())
			{
				result[1] = 2;
			}
			prbBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
	case 0x66:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Run Time\n");
            uint32_t tmp = 0;
			if (mode == MOCO_MODE_SMS)
			{
			    tmp = __builtin_bswap32(ulModeSmsGetCurrentTime());
			}
			else if (mode == MOCO_MODE_VIDEO_CONT)
			{
			    tmp = __builtin_bswap32(ulModeVideoGetCurrentTime());
			}
			else if (mode == MOCO_MODE_PANO)
			{
			}
			else if (mode == MOCO_MODE_ASTRO)
			{
			}
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			prbBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
	case 0x76:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get SMS / Continuous Program Mode\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, mode };
			prbBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
	case 0x7A:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Joystick Watchdog Mode Status\n");
			joystick_wdg_trigger = 0;
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			result[1] = joystick_wdg_enabled;
			prbBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
	case 0x7B:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Program %% Complete\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			if (mode == MOCO_MODE_SMS)
			{
			    result[1] = ucModeSmsGetProgress();
			}
			else if (mode == MOCO_MODE_VIDEO_CONT)
			{
			    result[1] = ucModeVideoGetProgress();
			}
			else if (mode == MOCO_MODE_PANO)
			{
			    result[1] = ucModePanoGetProgress();
			}
			else if (mode == MOCO_MODE_ASTRO)
			{
			}
			prbBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
	case 0x7D:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Total Program Run Time\n");
            uint32_t tmp = 0;
            if (mode == MOCO_MODE_SMS)
            {
                tmp = __builtin_bswap32(ulModeSmsGetOverallTime());
            }
            else if (mode == MOCO_MODE_VIDEO_CONT)
            {
                tmp = __builtin_bswap32(ulModeVideoGetOverallTime());
            }
            else if (mode == MOCO_MODE_PANO)
            {
                //tmp = __builtin_bswap32(ulModePanoGetOverallTime());
            }
            else if (mode == MOCO_MODE_ASTRO)
            {
            }
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			prbBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
	case 0x7E:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Program Complete?\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			if (bModeSmsIsFinished() || bModeVideoIsFinished() || bModePanoIsFinished() || bModeAstroIsFinished())
			{
				result[1] = 1;
			}
			else
			{
				result[1] = 0;
			}
			prbBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
	}

	prvBleUpdateMoCoControlPointTxOk();
	return false;
}

bool prbBleProcessMoCoControlPointRxMotor(uint8_t motor, uint8_t cmd, uint8_t * data, uint8_t data_length)
{
	switch (cmd)
	{
	case 0x02:
		{
			uint8_t power_save = data[0];
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Motor Sleep %d\n", motor, power_save);
			eep_params.sm[motor].power_save = power_save == 1 ? 1 : 2;
		
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x03:
		{
			uint8_t enable = data[0];
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Motor Enable %d\n", motor, enable);
			if (eep_params.sm[motor].power_save == 1) vSmEnable(motor, enable);
		
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x04:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Stop\n", motor);
			vSmStop(motor);
			
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x06:
		{
			uint8_t microsteps = data[0];
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Set Microstep Value\n", motor);
			if (microsteps == 4)
			{
				eep_params.sm[motor].microstep_mode = SM_MODE_STEALTH | SM_MODE_INTERPOLATION | SM_MODE_STEPS_DEFAULT;
			}
			else //if (microsteps == 8)
			{
				eep_params.sm[motor].microstep_mode = SM_MODE_INTERPOLATION | SM_MODE_STEPS_DEFAULT;
			}
			/*else
			{
				eep_params.sm[motor].microstep_mode = SM_MODE_STEPS_DEFAULT;
			}*/
			vSmSetMicrostepMode(motor, eep_params.sm[motor].microstep_mode);
		
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x0D:
		{
			uint32_t tmp;
			memcpy(&tmp, data, 4);
			tmp = __builtin_bswap32(tmp);
			float fspeed;
			memcpy(&fspeed, &tmp, 4);
			int32_t speed = (int32_t) fspeed;

			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Move Continuous %d\n", motor, speed);
			
			int32_t max_speed = lSmGetMaxSpeed(motor);
			
			joystick_wdg_trigger = 0;
			if (eep_params.sm[motor].power_save == 0) vSmEnable(motor, 1);
			if (eep_params.sm[motor].power_save == 2) vSmEnable(motor, 2);
			if (speed >= 0)
			{
				bSmMoveContinuous(motor, (int32_t) SM_STEPS_TO_MRAD(labs(speed)) * max_speed / (int32_t) SM_STEPS_TO_MRAD(5000));
			}
			else
			{
				bSmMoveContinuous(motor, -1 * (int32_t) SM_STEPS_TO_MRAD(labs(speed)) * max_speed / (int32_t) SM_STEPS_TO_MRAD(5000));
			}
			
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x13:
		{
			uint32_t lead_in = __builtin_bswap32(*(uint32_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Set Lead-In Shots / Time %d\n", motor, lead_in);
			eep_params.mode_sms_leadin_count[motor] = lead_in;
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x14:
		{
			uint32_t travel = __builtin_bswap32(*(uint32_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Set Travel Shots (SMS) / Time (Cont.) %d\n", motor, travel);
			if (travel != 0) eep_params.mode_video_duration[motor] = travel;
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x15:
		{
			uint32_t accel = __builtin_bswap32(*(uint32_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Set Accel Shots / Time %d\n", motor, accel);
			eep_params.mode_sms_accel_count[motor] = accel;
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x16:
		{
			uint32_t decel = __builtin_bswap32(*(uint32_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Set Decel Shots / Time %d\n", motor, decel);
			eep_params.mode_sms_decel_count[motor] = decel;
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x19:
		{
			uint32_t lead_out = __builtin_bswap32(*(uint32_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Set Lead-Out Shots / Time %d\n", motor, lead_out);
			eep_params.mode_sms_leadout_count[motor] = lead_out;
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x1B:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Reset Limits and Program Start/Stop Positions\n", motor);
			
			vSmResetPosition(motor);
			eep_params.mode_sms_positions[0].pos[motor] = 0;
			eep_params.mode_sms_positions[1].pos[motor] = 0;
			eep_params.mode_pano_position_start.pos[motor] = 0;
			eep_params.mode_pano_position_stop.pos[motor] = 0;
			eep_params.mode_pano_position_ref_start.pos[motor] = 0;
			eep_params.mode_pano_position_ref_stop.pos[motor] = 0;
			eep_params.mode_video_duration[motor] = 0;
		
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x1C:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Auto Set Program Microsteps\n", motor);
			uint8_t result[] = { MOCO_VALUE_BYTE, 16 };
			prbBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
	case 0x1D:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Set Start Here\n", motor);
			eep_params.mode_pano_position_ref_start.pos[motor] = lSmGetPosition(motor);
		
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x1E:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Set Stop Here\n", motor);
			eep_params.mode_pano_position_ref_stop.pos[motor] = lSmGetPosition(motor);
		
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x66:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Microstep Value\n", motor);
			uint8_t stealth = eep_params.sm[motor].microstep_mode & SM_MODE_STEALTH;
			uint8_t interpolation = eep_params.sm[motor].microstep_mode & SM_MODE_INTERPOLATION;
			
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			if (stealth)
			{
				result[1] = 4;
			}
			else if (interpolation)
			{
				result[1] = 8;
			}
			else
			{
				result[1] = 16;
			}
			prbBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
	case 0x6B:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Motor Running\n", motor);
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			result[1] = ucSmGetState(motor) != SM_STATE_STOP ? 1 : 0;
			prbBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
	case 0x6F:
	    {
    	    SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Program Start Point\n", motor);
    	    int32_t tmp = __builtin_bswap32(eep_params.mode_sms_positions[0].pos[motor]);
    	    uint8_t result[] = { MOCO_VALUE_LONG, 0, 0, 0, 0 };
    	    memcpy(&result[1], &tmp, 4);
    	    prbBleUpdateMoCoControlPointTxOkData(result, 5);
    	    return true;
	    }
	case 0x70:
	    {
    	    SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Program Stop Point\n", motor);
    	    int32_t tmp = __builtin_bswap32(eep_params.mode_sms_positions[1].pos[motor]);
    	    uint8_t result[] = { MOCO_VALUE_LONG, 0, 0, 0, 0 };
    	    memcpy(&result[1], &tmp, 4);
    	    prbBleUpdateMoCoControlPointTxOkData(result, 5);
    	    return true;
	    }
	case 0x71:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Travel Shots (SMS) / Time (Cont.)\n", motor);
            uint32_t tmp = 0;
            if (mode == MOCO_MODE_SMS)
            {
                tmp = __builtin_bswap32(eep_params.mode_video_duration[motor]);
            }
            else if (mode == MOCO_MODE_VIDEO_CONT)
            {
                tmp = __builtin_bswap32(eep_params.mode_video_duration[motor]);
            }
            else if (mode == MOCO_MODE_PANO)
            {
                if (motor == 1) tmp = __builtin_bswap32(ulModePanoGetOverallCols());
                if (motor == 2) tmp = __builtin_bswap32(ulModePanoGetOverallRows());
            }
            else if (mode == MOCO_MODE_ASTRO)
            {
            }
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			prbBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
	case 0x72:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Lead-In Shots / Time\n", motor);
			uint32_t tmp = eep_params.mode_sms_leadin_count[motor];
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			prbBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
	case 0x73:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Accel Shots / Time\n", motor);
			uint32_t tmp = eep_params.mode_sms_accel_count[motor];
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			prbBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
	case 0x74:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Decel Shots / Time\n", motor);
			uint32_t tmp = eep_params.mode_sms_decel_count[motor];
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			prbBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
	case 0x75:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Check Motor Sleep State\n", motor);
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			result[1] = eep_params.sm[motor].power_save == 1 ? 1 : 0;
			prbBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
	case 0x77:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Lead-Out Shots / Time\n", motor);
			uint32_t tmp = eep_params.mode_sms_leadout_count[motor];
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			prbBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
	}

	prvBleUpdateMoCoControlPointTxOk();
	return false;
}

bool prbBleProcessMoCoControlPointRxCamera(uint8_t cmd, uint8_t * data, uint8_t data_length)
{
	switch (cmd)
	{
	case 0x03:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Expose Now\n");
			vModeSmsStartExposeNow();
		
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x04:
		{
			uint32_t trigger_time = __builtin_bswap32(*(uint32_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Trigger Time %d\n", trigger_time);
			eep_params.mode_sms_exposure_time = trigger_time;
		
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x05:
		{
			uint16_t focus_time = __builtin_bswap16(*(uint16_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Focus Time %d\n", focus_time);
			eep_params.mode_sms_focus_time = focus_time;
		
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x06:
		{
			uint16_t count = __builtin_bswap16(*(uint16_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Max Shots %d\n", count);
			eep_params.mode_sms_count = count;
						
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x07:
		{
			uint16_t exposure_delay_time = __builtin_bswap16(*(uint16_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Exposure Delay %d\n", exposure_delay_time);
			eep_params.mode_sms_post_time = exposure_delay_time;
		
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x0A:
		{
			uint32_t interval = __builtin_bswap32(*(uint32_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Interval %d\n", interval);
			eep_params.mode_sms_interval = interval;
						
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x0B:
		{
			uint8_t enable = data[0];
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Camera Test Mode\n");
			if (enable)
			{
				vModeSmsStartCameraTest();
			}
			else
			{
				vModeSmsStop();				
			}
		
			prvBleUpdateMoCoControlPointTxOk();
			return true;
		}
	case 0x64:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Camera Enable\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, 1 };
			prbBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
	case 0x65:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Exposing now?\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			prbBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
	case 0x66:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Trigger Time\n");
			uint32_t tmp = __builtin_bswap32(eep_params.mode_sms_exposure_time);
			uint8_t result[] = { MOCO_VALUE_LONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			prbBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
	case 0x67:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Focus Time\n");
			uint16_t tmp = __builtin_bswap16(eep_params.mode_sms_focus_time);
			uint8_t result[] = { MOCO_VALUE_UINT, 0, 0 };
			memcpy(&result[1], &tmp, 2);
			prbBleUpdateMoCoControlPointTxOkData(result, 3);
			return true;
		}
	case 0x68:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Max Shots\n");
			uint32_t tmp = 0;
            if (mode == MOCO_MODE_SMS)
            {
                tmp = __builtin_bswap32(eep_params.mode_sms_count);
            }
            else if (mode == MOCO_MODE_VIDEO_CONT)
            {
            }
            else if (mode == MOCO_MODE_PANO)
            {
                tmp = __builtin_bswap32(ulModePanoGetOverallCycles());
            }
            else if (mode == MOCO_MODE_ASTRO)
            {
            }
			uint8_t result[] = { MOCO_VALUE_LONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			prbBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
	case 0x69:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Exposure Delay\n");
			uint16_t tmp = __builtin_bswap16(eep_params.mode_sms_post_time);
			uint8_t result[] = { MOCO_VALUE_UINT, 0, 0 };
			memcpy(&result[1], &tmp, 2);
			prbBleUpdateMoCoControlPointTxOkData(result, 3);
			return true;
		}
	case 0x6C:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Interval\n");
			uint32_t tmp = __builtin_bswap32(eep_params.mode_sms_interval);
			uint8_t result[] = { MOCO_VALUE_LONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			prbBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
	case 0x6D:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Current Shots\n");
            uint16_t tmp = 0;
            if (mode == MOCO_MODE_SMS)
            {
                tmp = __builtin_bswap16((uint16_t)ulModeSmsGetCurrentCycle());
            }
            else if (mode == MOCO_MODE_VIDEO_CONT)
            {
            }
            else if (mode == MOCO_MODE_PANO)
            {
                tmp = __builtin_bswap16((uint16_t)ulModePanoGetCurrentCycle());
            }
            else if (mode == MOCO_MODE_ASTRO)
            {
            }
			uint8_t result[] = { MOCO_VALUE_UINT, 0, 0 };
			memcpy(&result[1], &tmp, 2);
			prbBleUpdateMoCoControlPointTxOkData(result, 3);
			return true;
		}
	case 0x6E:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Camera Test Mode\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			result[1] = ucModeSmsGetState() != MODE_SMS_STATE_STOP && bModeSmsGetCameraTestMode() ? 1 : 0;
			prbBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
	}
	
	prvBleUpdateMoCoControlPointTxOk();
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
									
									char * name = versionDEVICE_NAME_DEFAULT;
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
					
					/* check if the peer has subscribed to the Heart Rate Measurement Characteristic for Notifications	*/
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
								prbBleProcessMoCoControlPointRx(subadr, cmd, data, len);
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
