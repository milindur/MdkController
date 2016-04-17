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

bool bBleProcessMoCoControlPointRxMain(uint8_t cmd, uint8_t * data, uint8_t data_length)
{
	switch (cmd)
	{
		case 0x02:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Start Program\n");
			if (ucBleGetMode() == MOCO_MODE_SMS)
			{
				if (bModeSmsIsPaused())
				{
					SEGGER_RTT_printf(0, "MoCoBus Control RX: Start Program - Resume SMS\n");
					vModeSmsResume();
				}
				else
				{
					SEGGER_RTT_printf(0, "MoCoBus Control RX: Start Program - Start SMS\n");
					vModeSmsStart();
				}
			}
			else if (ucBleGetMode() == MOCO_MODE_VIDEO_CONT)
			{
				if (bModeVideoIsPaused())
				{
					SEGGER_RTT_printf(0, "MoCoBus Control RX: Start Program - Resume Video\n");
					vModeVideoResume();
				}
				else
				{
					SEGGER_RTT_printf(0, "MoCoBus Control RX: Start Program - Start Video\n");
					vModeVideoStart();
				}
			}
			else if (ucBleGetMode() == MOCO_MODE_PANO)
			{
				if (bModePanoIsPaused())
				{
					SEGGER_RTT_printf(0, "MoCoBus Control RX: Start Program - Resume Pano\n");
					vModePanoResume();
				}
				else
				{
					SEGGER_RTT_printf(0, "MoCoBus Control RX: Start Program - Start Pano\n");
					vModePanoStart();
				}
			}
			else if (ucBleGetMode() == MOCO_MODE_ASTRO)
			{
				if (bModeAstroIsPaused())
				{
					SEGGER_RTT_printf(0, "MoCoBus Control RX: Start Program - Resume Astro\n");
					vModeAstroResume();
				}
				else
				{
					if (data_length == 2)
					{
						uint8_t dir = data[0] == 0 ? MODE_ASTRO_DIR_NORTH : MODE_ASTRO_DIR_SOUTH;
						uint8_t spd = data[1] == 0 ? MODE_ASTRO_SPEED_SIDEREAL : MODE_ASTRO_SPEED_LUNAR;
						SEGGER_RTT_printf(0, "MoCoBus Control RX: Start Program - Start Astro %d / %d\n", dir, spd);
						vModeAstroStart(dir, spd);
					}
					else if (data_length == 1)
					{
						uint8_t dir = data[0] == 0 ? MODE_ASTRO_DIR_NORTH : MODE_ASTRO_DIR_SOUTH;
						SEGGER_RTT_printf(0, "MoCoBus Control RX: Start Program - Start Astro %d\n", dir);
						vModeAstroStart(dir, MODE_ASTRO_SPEED_SIDEREAL);
					}
					else
					{
						SEGGER_RTT_printf(0, "MoCoBus Control RX: Start Program - Start Astro\n");
						vModeAstroStart(MODE_ASTRO_DIR_NORTH, MODE_ASTRO_SPEED_SIDEREAL);
					}
				}
			}
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x03:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Pause Program\n");
			if (ucBleGetMode() == MOCO_MODE_SMS)
			{
				vModeSmsPause();
			}
			else if (ucBleGetMode() == MOCO_MODE_VIDEO_CONT)
			{
				vModeVideoPause();
			}
			else if (ucBleGetMode() == MOCO_MODE_PANO)
			{
				vModePanoPause();
			}
			else if (ucBleGetMode() == MOCO_MODE_ASTRO)
			{
				vModeAstroPause();
			}
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x04:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Stop Program\n");
			if (ucBleGetMode() == MOCO_MODE_SMS)
			{
				vModeSmsStop();
			}
			else if (ucBleGetMode() == MOCO_MODE_VIDEO_CONT)
			{
				vModeVideoStop();
			}
			else if (ucBleGetMode() == MOCO_MODE_PANO)
			{
				vModePanoStop();
			}
			else if (ucBleGetMode() == MOCO_MODE_ASTRO)
			{
				vModeAstroStop();
			}
			
			vBleUpdateMoCoControlPointTxOk();
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
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x0E:
		{
			uint8_t enable = data[0];
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Set Joystick Watchdog %d\n", enable);
			vBleSetJoystickWatchdog(enable);
			vBleJoystickTriggerReset();
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x16:
		{
			if (!(bModeSmsIsPaused() || bModeVideoIsPaused() || bModePanoIsPaused() || bModeAstroIsPaused()
			|| bModeSmsIsRunning() || bModeVideoIsRunning() || bModePanoIsRunning() || bModeAstroIsRunning()))
			{
				vBleSetMode(data[0]);
			}
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Set Program Mode %d\n", ucBleGetMode());
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x18:
		{
			uint8_t ping_pong = data[0];
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Set Ping-Pong Flag %d\n", ping_pong);
			eep_params.mode_video_ping_pong = ping_pong;
			vBleUpdateMoCoControlPointTxOk();
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
				if (ucBleGetMode() == MOCO_MODE_SMS || ucBleGetMode() == MOCO_MODE_VIDEO_CONT)
				{
					steps = eep_params.mode_sms_positions[0].pos[motor] - lSmGetPosition(motor);
				}
				else if (ucBleGetMode() == MOCO_MODE_PANO)
				{
					steps = eep_params.mode_pano_position_start.pos[motor] - lSmGetPosition(motor);
				}
				ucSmMove(motor, steps);
			}
			
			vBleUpdateMoCoControlPointTxOk();
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
			
			vBleUpdateMoCoControlPointTxOk();
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
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x1D:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Swap Program Start/End Points\n");
			
			if (ucBleGetMode() == MOCO_MODE_SMS || ucBleGetMode() == MOCO_MODE_VIDEO_CONT)
			{
				for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
				{
					vUtilsSwap(&eep_params.mode_sms_positions[0].pos[motor], &eep_params.mode_sms_positions[1].pos[motor]);
				}
			}
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x32:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Set Graffik Mode\n");
			vBleUpdateMoCoControlPointTxOk();
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
			bBleUpdateMoCoControlPointTxOkData(result, 5);
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
			bBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
		case 0x66:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Run Time\n");
			uint32_t tmp = 0;
			if (ucBleGetMode() == MOCO_MODE_SMS)
			{
				tmp = __builtin_bswap32(ulModeSmsGetCurrentTime());
			}
			else if (ucBleGetMode() == MOCO_MODE_VIDEO_CONT)
			{
				tmp = __builtin_bswap32(ulModeVideoGetCurrentTime());
			}
			else if (ucBleGetMode() == MOCO_MODE_PANO)
			{
			}
			else if (ucBleGetMode() == MOCO_MODE_ASTRO)
			{
			}
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			bBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
		case 0x76:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get SMS / Continuous Program Mode\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, ucBleGetMode() };
			bBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
		case 0x7A:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Joystick Watchdog Mode Status\n");
			vBleJoystickTriggerReset();
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			result[1] = ucBleGetJoystickWatchdog();
			bBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
		case 0x7B:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Program %% Complete\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			if (ucBleGetMode() == MOCO_MODE_SMS)
			{
				result[1] = ucModeSmsGetProgress();
			}
			else if (ucBleGetMode() == MOCO_MODE_VIDEO_CONT)
			{
				result[1] = ucModeVideoGetProgress();
			}
			else if (ucBleGetMode() == MOCO_MODE_PANO)
			{
				result[1] = ucModePanoGetProgress();
			}
			else if (ucBleGetMode() == MOCO_MODE_ASTRO)
			{
			}
			bBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
		case 0x7D:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Total Program Run Time\n");
			uint32_t tmp = 0;
			if (ucBleGetMode() == MOCO_MODE_SMS)
			{
				tmp = __builtin_bswap32(ulModeSmsGetOverallTime());
			}
			else if (ucBleGetMode() == MOCO_MODE_VIDEO_CONT)
			{
				tmp = __builtin_bswap32(ulModeVideoGetOverallTime());
			}
			else if (ucBleGetMode() == MOCO_MODE_PANO)
			{
				//tmp = __builtin_bswap32(ulModePanoGetOverallTime());
			}
			else if (ucBleGetMode() == MOCO_MODE_ASTRO)
			{
			}
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			bBleUpdateMoCoControlPointTxOkData(result, 5);
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
			bBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
	}

	vBleUpdateMoCoControlPointTxOk();
	return false;
}
