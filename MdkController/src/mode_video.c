/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* Atmel library includes. */
#include "asf.h"

#include "SEGGER_RTT.h"
#include "eep.h"
#include "sm.h"
#include "ble.h"
#include "mode_video.h"
#include "utils.h"

#define mode_videoCONTROL_TIMER_RATE     (10 / portTICK_RATE_MS)

static uint8_t state = MODE_VIDEO_STATE_STOP;
static bool finished = false;
static bool start_to_end = true;
static uint32_t step_timer;
static uint32_t remaining_time;
static uint32_t overall_time;
static uint32_t elapsed_time;
static xTimerHandle xModeVideoControlTimer;

static void prvModeVideoControlCallback(void *pvParameters);

void vModeVideoInit(void)
{
#if !defined(DEBUG)
	for (uint8_t f = 0; f < MODE_VIDEO_MAX_KEY_FRAMES; f++)
	{
		for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
		{
			eep_params.mode_sms_positions[f].pos[motor] = 0;
		}
	}
#endif

	xModeVideoControlTimer = xTimerCreate(
		(const char * const) "ModeVideoCtrlTmr",
		mode_videoCONTROL_TIMER_RATE,
		pdTRUE,
		NULL,
		prvModeVideoControlCallback);
	configASSERT(xModeVideoControlTimer);
}

void vModeVideoSetStartEnd(int32_t start, int32_t end)
{
    eep_params.mode_sms_positions[0].pos[0] = start;
    eep_params.mode_sms_positions[1].pos[0] = end;

#if DEBUG
    vEepSave();
#endif
}

void vModeVideoGetStartEnd(int32_t *start, int32_t *end)
{
    *start = eep_params.mode_sms_positions[0].pos[0];
    *end = eep_params.mode_sms_positions[1].pos[0];
}

void vModeVideoPause(void)
{
	if (state == MODE_VIDEO_STATE_STOP) return;
	if (bModeVideoIsPaused()) return;
	
	xTimerStop(xModeVideoControlTimer, 0);
}

void vModeVideoResume(void)
{
	if (state == MODE_VIDEO_STATE_STOP) return;
	if (!bModeVideoIsPaused()) return;
	
	xTimerStart(xModeVideoControlTimer, 0);
}

void vModeVideoStart(void)
{
	if (state != MODE_VIDEO_STATE_STOP) return;
	
	taskENTER_CRITICAL();
	{
		step_timer = 0;
   
        vModeVideoCalcTime();
		
		overall_time = eep_params.mode_sms_count * eep_params.mode_sms_interval;

		state = MODE_VIDEO_STATE_WAKE_SM;
		xTimerStart(xModeVideoControlTimer, 0);
	}
	taskEXIT_CRITICAL();
}

void vModeVideoStop(void)
{
    taskENTER_CRITICAL();
    {
		xTimerStop(xModeVideoControlTimer, 0);
		
		for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
		{
			vSmStop(motor);
			if (eep_params.sm[motor].power_save == 0) vSmEnable(motor, 0);
		}
		state = MODE_VIDEO_STATE_STOP;
		finished = true;
	}
	taskEXIT_CRITICAL();
}

bool bModeVideoIsFinished(void)
{
	bool v;
	
	taskENTER_CRITICAL();
	{
		v = finished;
		finished = false;
	}
	taskEXIT_CRITICAL();
	
	return v;
}

bool bModeVideoIsPaused(void)
{
	if (state == MODE_VIDEO_STATE_STOP) return false;
	
	return xTimerIsTimerActive(xModeVideoControlTimer) == pdFALSE;
}

bool bModeVideoIsRunning(void)
{
	return state != MODE_VIDEO_STATE_STOP;
}

uint8_t ucModeVideoGetState(void)
{
	uint8_t v;
	
    taskENTER_CRITICAL();
    {
		v = state;
	}
	taskEXIT_CRITICAL();
	
	return v;
}

uint8_t ucModeVideoGetProgress(void)
{
	return 0;
}

uint32_t ulModeVideoGetRemainingTime(void)
{
	uint32_t v;
	
	taskENTER_CRITICAL();
	{
		v = remaining_time;
	}
	taskEXIT_CRITICAL();
	
	return v;
}

uint32_t ulModeVideoGetCurrentTime(void)
{
	uint32_t v;
	
	taskENTER_CRITICAL();
	{
		v = elapsed_time;
	}
	taskEXIT_CRITICAL();
	
	return v;
}

uint32_t ulModeVideoGetOverallTime(void)
{
	return overall_time;
}

void vModeVideoCalcTime(void)
{
    //remaining_time = (eep_params.mode_sms_count - current_loop) * eep_params.mode_sms_interval - interval_timer;
	//elapsed_time = current_loop * eep_params.mode_sms_interval + interval_timer;
}

static void prvModeVideoControlCallback(void *pvParameters)
{
	if (state >= MODE_VIDEO_STATE_MOVE && state <= MODE_VIDEO_STATE_WAIT_MOVE)
	{
		step_timer += 10;
	}

	switch (state)
	{
		case MODE_VIDEO_STATE_STOP:
			xTimerStop(xModeVideoControlTimer, 0);
			break;
		case MODE_VIDEO_STATE_WAKE_SM:
			{
				for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
				{
					if (eep_params.sm[motor].power_save == 0) vSmEnable(motor, 1);
					if (eep_params.sm[motor].power_save == 2) vSmEnable(motor, 2);
				}
			}
            state = MODE_VIDEO_STATE_GOTO_START;
			SEGGER_RTT_printf(0, "ModeVideo Control State Change: GOTO_START\n");
			break;
		case MODE_VIDEO_STATE_GOTO_START:
			{
				for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
				{
					ucSmMove(motor, eep_params.mode_sms_positions[0].pos[motor] - lSmGetPosition(motor));
				}
			}
            state = MODE_VIDEO_STATE_WAIT_START;
			SEGGER_RTT_printf(0, "ModeVideo Control State Change: WAIT_START\n");
		    break;
		case MODE_VIDEO_STATE_WAIT_START:
            if ((ucSmGetState(0) == SM_STATE_STOP) && (ucSmGetState(1) == SM_STATE_STOP) && (ucSmGetState(2) == SM_STATE_STOP))
            {
                state = MODE_VIDEO_STATE_MOVE;
				SEGGER_RTT_printf(0, "ModeVideo Control State Change: WAIT_PRE_TIME\n");
                step_timer = 0;
				start_to_end = true;
            }                
		    break;
		case MODE_VIDEO_STATE_MOVE:
            {
				for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
				{
					// calculate available time for video movement
					int32_t avail_move_time = eep_params.mode_video_duration[motor];
				
					// sanity check
					if (avail_move_time < 100) avail_move_time = 100;

					int32_t steps = 0;
					if (start_to_end)
					{
						steps = eep_params.mode_sms_positions[1].pos[motor] - lSmGetPosition(motor);
					}
					else
					{
						steps = eep_params.mode_sms_positions[0].pos[motor] - lSmGetPosition(motor);
					}
				
					if (eep_params.sm[motor].power_save == 1) vSmEnable(motor, 1);
					
					uint16_t accel = SM_STEPS_TO_MRAD(eep_params.sm[motor].accel_steps);
					uint16_t max_speed = SM_STEPS_TO_MRAD(abs(steps) * 1000 / avail_move_time);
					ucSmMoveEx(motor, steps, max_speed, accel, accel);
				}
				
				state = MODE_VIDEO_STATE_WAIT_MOVE;
				SEGGER_RTT_printf(0, "ModeVideo Control State Change: WAIT_MOVE\n");
            }
			break;
		case MODE_VIDEO_STATE_WAIT_MOVE:
			{
				bool all_stoped = true;
				for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
				{
					if (ucSmGetState(motor) != SM_STATE_STOP)
					{
						all_stoped = false;
						continue;
					}
					if (eep_params.sm[motor].power_save == 1) vSmEnable(motor, 0);
				}
				if (all_stoped)
				{
					if (eep_params.mode_video_ping_pong)
					{
						start_to_end = !start_to_end;
						state = MODE_VIDEO_STATE_MOVE;
						SEGGER_RTT_printf(0, "ModeVideo Control State Change: MOVE\n");
					}
					else
					{
						state = MODE_VIDEO_STATE_GOTO_END;
						SEGGER_RTT_printf(0, "ModeVideo Control State Change: GOTO_END\n");
					}
				}
			}
			break;
		case MODE_VIDEO_STATE_GOTO_END:
			{
				for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
				{
					if (eep_params.sm[motor].power_save == 1) vSmEnable(motor, 1);
					ucSmMove(motor, eep_params.mode_sms_positions[1].pos[motor] - lSmGetPosition(motor));
				}
			}
			state = MODE_VIDEO_STATE_WAIT_END;
			SEGGER_RTT_printf(0, "ModeVideo Control State Change: WAIT_END\n");
			break;
		case MODE_VIDEO_STATE_WAIT_END:
			{
				bool all_stoped = true;
				for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
				{
					if (ucSmGetState(motor) != SM_STATE_STOP)
					{
						all_stoped = false;
						continue;
					}
					if (eep_params.sm[motor].power_save == 1) vSmEnable(motor, 0);
				}
				if (all_stoped)
				{
					state = MODE_VIDEO_STATE_SLEEP_SM;
					SEGGER_RTT_printf(0, "ModeVideo Control State Change: SLEEP_SM\n");
					step_timer = 0;
				}
			}
			break;
		case MODE_VIDEO_STATE_SLEEP_SM:
			{
				for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
				{
					if (eep_params.sm[motor].power_save == 1) vSmEnable(motor, 0);
				}
			}
			state = MODE_VIDEO_STATE_STOP;
			finished = true;
			SEGGER_RTT_printf(0, "ModeVideo Control State Change: STOP\n");
			break;
	}

	vModeVideoCalcTime();
}
