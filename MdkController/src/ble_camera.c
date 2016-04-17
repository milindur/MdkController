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

bool bBleProcessMoCoControlPointRxCamera(uint8_t cmd, uint8_t * data, uint8_t data_length)
{
	switch (cmd)
	{
		case 0x03:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Expose Now\n");
			vModeSmsStartExposeNow();
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x04:
		{
			uint32_t trigger_time = __builtin_bswap32(*(uint32_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Trigger Time %d\n", trigger_time);
			eep_params.mode_sms_exposure_time = trigger_time;
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x05:
		{
			uint16_t focus_time = __builtin_bswap16(*(uint16_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Focus Time %d\n", focus_time);
			eep_params.mode_sms_focus_time = focus_time;
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x06:
		{
			uint16_t count = __builtin_bswap16(*(uint16_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Max Shots %d\n", count);
			eep_params.mode_sms_count = count;
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x07:
		{
			uint16_t exposure_delay_time = __builtin_bswap16(*(uint16_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Exposure Delay %d\n", exposure_delay_time);
			eep_params.mode_sms_post_time = exposure_delay_time;
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x0A:
		{
			uint32_t interval = __builtin_bswap32(*(uint32_t *)data);
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Interval %d\n", interval);
			eep_params.mode_sms_interval = interval;
			
			vBleUpdateMoCoControlPointTxOk();
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
			
			vBleUpdateMoCoControlPointTxOk();
			return true;
		}
		case 0x64:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Camera Enable\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, 1 };
			bBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
		case 0x65:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Exposing now?\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			bBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
		case 0x66:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Trigger Time\n");
			uint32_t tmp = __builtin_bswap32(eep_params.mode_sms_exposure_time);
			uint8_t result[] = { MOCO_VALUE_LONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			bBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
		case 0x67:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Focus Time\n");
			uint16_t tmp = __builtin_bswap16(eep_params.mode_sms_focus_time);
			uint8_t result[] = { MOCO_VALUE_UINT, 0, 0 };
			memcpy(&result[1], &tmp, 2);
			bBleUpdateMoCoControlPointTxOkData(result, 3);
			return true;
		}
		case 0x68:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Max Shots\n");
			uint32_t tmp = 0;
			if (ucBleGetMode() == MOCO_MODE_SMS)
			{
				tmp = __builtin_bswap32(eep_params.mode_sms_count);
			}
			else if (ucBleGetMode() == MOCO_MODE_VIDEO_CONT)
			{
			}
			else if (ucBleGetMode() == MOCO_MODE_PANO)
			{
				tmp = __builtin_bswap32(ulModePanoGetOverallCycles());
			}
			else if (ucBleGetMode() == MOCO_MODE_ASTRO)
			{
			}
			uint8_t result[] = { MOCO_VALUE_LONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			bBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
		case 0x69:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Exposure Delay\n");
			uint16_t tmp = __builtin_bswap16(eep_params.mode_sms_post_time);
			uint8_t result[] = { MOCO_VALUE_UINT, 0, 0 };
			memcpy(&result[1], &tmp, 2);
			bBleUpdateMoCoControlPointTxOkData(result, 3);
			return true;
		}
		case 0x6C:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Interval\n");
			uint32_t tmp = __builtin_bswap32(eep_params.mode_sms_interval);
			uint8_t result[] = { MOCO_VALUE_LONG, 0, 0, 0, 0 };
			memcpy(&result[1], &tmp, 4);
			bBleUpdateMoCoControlPointTxOkData(result, 5);
			return true;
		}
		case 0x6D:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Current Shots\n");
			uint16_t tmp = 0;
			if (ucBleGetMode() == MOCO_MODE_SMS)
			{
				tmp = __builtin_bswap16((uint16_t)ulModeSmsGetCurrentCycle());
			}
			else if (ucBleGetMode() == MOCO_MODE_VIDEO_CONT)
			{
			}
			else if (ucBleGetMode() == MOCO_MODE_PANO)
			{
				tmp = __builtin_bswap16((uint16_t)ulModePanoGetCurrentCycle());
			}
			else if (ucBleGetMode() == MOCO_MODE_ASTRO)
			{
			}
			uint8_t result[] = { MOCO_VALUE_UINT, 0, 0 };
			memcpy(&result[1], &tmp, 2);
			bBleUpdateMoCoControlPointTxOkData(result, 3);
			return true;
		}
		case 0x6E:
		{
			SEGGER_RTT_printf(0, "MoCoBus Control RX: Get Camera Test Mode\n");
			uint8_t result[] = { MOCO_VALUE_BYTE, 0 };
			result[1] = ucModeSmsGetState() != MODE_SMS_STATE_STOP && bModeSmsGetCameraTestMode() ? 1 : 0;
			bBleUpdateMoCoControlPointTxOkData(result, 2);
			return true;
		}
	}
	
	vBleUpdateMoCoControlPointTxOk();
	return false;
}
