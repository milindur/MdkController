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

bool bBleProcessMoCoControlPointRxMotor(uint8_t motor, uint8_t cmd, uint8_t * data, uint8_t data_length)
{
	switch (cmd)
	{
		case 0x02:
		{
			uint8_t power_save = data[0];
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Motor Sleep %d\n", motor, power_save);
			eep_params.sm[motor].power_save = power_save == 1 ? 1 : 2;
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x03:
		{
			uint8_t enable = data[0];
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Motor Enable %d\n", motor, enable);
			if (eep_params.sm[motor].power_save == 1) vSmEnable(motor, enable);
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x04:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Stop\n", motor);
			vSmStop(motor);
			
			vBleUpdateMoCoControlPointTxOk();
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
			
			vBleUpdateMoCoControlPointTxOk();
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
			
			vBleJoystickTriggerReset();
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
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x13:
		{
			uint32_t lead_in = __builtin_bswap32(*(uint32_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Set Lead-In Shots / Time %d\n", motor, lead_in);
			eep_params.mode_sms_leadin_count[motor] = lead_in;
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x14:
		{
			uint32_t travel = __builtin_bswap32(*(uint32_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Set Travel Shots (SMS) / Time (Cont.) %d\n", motor, travel);
			if (travel != 0) eep_params.mode_video_duration[motor] = travel;
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x15:
		{
			uint32_t accel = __builtin_bswap32(*(uint32_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Set Accel Shots / Time %d\n", motor, accel);
			eep_params.mode_sms_accel_count[motor] = accel;
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x16:
		{
			uint32_t decel = __builtin_bswap32(*(uint32_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Set Decel Shots / Time %d\n", motor, decel);
			eep_params.mode_sms_decel_count[motor] = decel;
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x19:
		{
			uint32_t lead_out = __builtin_bswap32(*(uint32_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Set Lead-Out Shots / Time %d\n", motor, lead_out);
			eep_params.mode_sms_leadout_count[motor] = lead_out;
			vBleUpdateMoCoControlPointTxOk();
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
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x1C:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Auto Set Program Microsteps\n", motor);
			uint8_t result[] = { MOCO_VALUE_BYTE, 16 };
			bBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
		case 0x1D:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Set Start Here\n", motor);
			eep_params.mode_pano_position_ref_start.pos[motor] = lSmGetPosition(motor);
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x1E:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Set Stop Here\n", motor);
			eep_params.mode_pano_position_ref_stop.pos[motor] = lSmGetPosition(motor);
			
			vBleUpdateMoCoControlPointTxOk();
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
			bBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
		case 0x6B:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Motor Running\n", motor);
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			result[1] = ucSmGetState(motor) != SM_STATE_STOP ? 1 : 0;
			bBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
		case 0x6F:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Program Start Point\n", motor);
			int32_t tmp = __builtin_bswap32(eep_params.mode_sms_positions[0].pos[motor]);
			uint8_t result[] = { MOCO_VALUE_LONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			bBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
		case 0x70:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Program Stop Point\n", motor);
			int32_t tmp = __builtin_bswap32(eep_params.mode_sms_positions[1].pos[motor]);
			uint8_t result[] = { MOCO_VALUE_LONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			bBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
		case 0x71:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Travel Shots (SMS) / Time (Cont.)\n", motor);
			uint32_t tmp = 0;
			if (ucBleGetMode() == MOCO_MODE_SMS)
			{
				tmp = __builtin_bswap32(eep_params.mode_video_duration[motor]);
			}
			else if (ucBleGetMode() == MOCO_MODE_VIDEO_CONT)
			{
				tmp = __builtin_bswap32(eep_params.mode_video_duration[motor]);
			}
			else if (ucBleGetMode() == MOCO_MODE_PANO)
			{
				if (motor == 1) tmp = __builtin_bswap32(ulModePanoGetOverallCols());
				if (motor == 2) tmp = __builtin_bswap32(ulModePanoGetOverallRows());
			}
			else if (ucBleGetMode() == MOCO_MODE_ASTRO)
			{
			}
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			bBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
		case 0x72:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Lead-In Shots / Time\n", motor);
			uint32_t tmp = eep_params.mode_sms_leadin_count[motor];
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			bBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
		case 0x73:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Accel Shots / Time\n", motor);
			uint32_t tmp = eep_params.mode_sms_accel_count[motor];
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			bBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
		case 0x74:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Decel Shots / Time\n", motor);
			uint32_t tmp = eep_params.mode_sms_decel_count[motor];
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			bBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
		case 0x75:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Check Motor Sleep State\n", motor);
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			result[1] = eep_params.sm[motor].power_save == 1 ? 1 : 0;
			bBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
		case 0x77:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: [MOTOR%d] Get Lead-Out Shots / Time\n", motor);
			uint32_t tmp = eep_params.mode_sms_leadout_count[motor];
			uint8_t result[] = { MOCO_VALUE_ULONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			bBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
	}

	vBleUpdateMoCoControlPointTxOk();
	return false;
}
