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
#include "cam.h"
#include "ble.h"
#include "mode_sms.h"
#include "utils.h"

#define mode_smsTEST_NONE			0
#define mode_smsTEST_CAMERA		1
#define mode_smsTEST_EXPOSE_NOW	2

#define mode_smsCONTROL_TIMER_RATE     (10 / portTICK_RATE_MS)

static uint8_t state = MODE_SMS_STATE_STOP;
static bool finished = false;
static uint8_t test_mode = mode_smsTEST_NONE;
static uint32_t current_step;
static uint32_t current_loop;
static int32_t ramp_steps_x1000[SM_MOTORS_USED];
static int32_t run_steps[SM_MOTORS_USED];
static uint32_t step_timer;
static uint32_t interval_timer;
static uint32_t remaining_step_time;
static uint32_t remaining_time;
static uint32_t overall_time;
static uint32_t elapsed_time;
static xTimerHandle xModeSmsControlTimer;

static void prvModeSmsControlCallback(void *pvParameters);

void vModeSmsInit(void)
{
#if !defined(DEBUG)
	for (uint8_t f = 0; f < MODE_SMS_MAX_KEY_FRAMES; f++)
	{
		for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
		{
			eep_params.mode_sms_positions[f].pos[motor] = 0;
		}
	}
#endif

	xModeSmsControlTimer = xTimerCreate(
		(const char * const) "ModeSmsCtrlTmr",
		mode_smsCONTROL_TIMER_RATE,
		pdTRUE,
		NULL,
		prvModeSmsControlCallback);
	configASSERT(xModeSmsControlTimer);
}

void vModeSmsSetParams(mode_sms_setup_t *params)
{
	eep_params.mode_sms_pre_time = params->pre_time;
	eep_params.mode_sms_focus_time = params->focus_time;
	eep_params.mode_sms_exposure_time = params->exposure_time;
	eep_params.mode_sms_post_time = params->post_time;

    vModeSmsUpdateIntervalToMinimum();
}

void vModeSmsGetParams(mode_sms_setup_t *params)
{
	params->pre_time = eep_params.mode_sms_pre_time;
	params->focus_time = eep_params.mode_sms_focus_time;
	params->exposure_time = eep_params.mode_sms_exposure_time;
	params->post_time = eep_params.mode_sms_post_time;
}

void vModeSmsSetStartEnd(int32_t start, int32_t end)
{
    eep_params.mode_sms_positions[0].pos[0] = start;
    eep_params.mode_sms_positions[1].pos[0] = end;

#if DEBUG
    vEepSave();
#endif
}

void vModeSmsGetStartEnd(int32_t *start, int32_t *end)
{
    *start = eep_params.mode_sms_positions[0].pos[0];
    *end = eep_params.mode_sms_positions[1].pos[0];
}

void vModeSmsSetInterval(uint32_t interval, uint32_t count, uint16_t ramp_count, uint16_t stall_count)
{
    eep_params.mode_sms_interval = interval;
    eep_params.mode_sms_count = count;
    eep_params.mode_sms_ramp_count = ramp_count;
    eep_params.mode_sms_stall_count = stall_count;
    vEepSave();
}

void vModeSmsGetInterval(uint32_t *interval, uint32_t *count, uint16_t *ramp_count, uint16_t *stall_count)
{
    *interval = eep_params.mode_sms_interval;
    *count = eep_params.mode_sms_count;
    *ramp_count = eep_params.mode_sms_ramp_count;
    *stall_count = eep_params.mode_sms_stall_count;
}

void vModeSmsUpdateIntervalToMinimum(void)
{
    eep_params.mode_sms_interval = utilsMAX(eep_params.mode_sms_interval, ulModeSmsGetMinimumInterval(eep_params.mode_sms_pre_time, eep_params.mode_sms_focus_time, eep_params.mode_sms_exposure_time, eep_params.mode_sms_post_time));
    vEepSave();
}

uint32_t ulModeSmsGetMinimumInterval(uint32_t pre_time, uint32_t focus_time, uint32_t exposure_time, uint32_t post_time)
{
    return pre_time + focus_time + exposure_time + post_time + 500;
}

void vModeSmsPause(void)
{
	if (state == MODE_SMS_STATE_STOP) return;
	if (bModeSmsGetPaused()) return;
	
	xTimerStop(xModeSmsControlTimer, 0);
}

void vModeSmsResume(void)
{
	if (state == MODE_SMS_STATE_STOP) return;
	if (!bModeSmsGetPaused()) return;
	
	xTimerStart(xModeSmsControlTimer, 0);
}

void vModeSmsStartExposeNow(void)
{
	if (state != MODE_SMS_STATE_STOP) return;

	taskENTER_CRITICAL();
	{
		test_mode = mode_smsTEST_EXPOSE_NOW;
		
		current_step = 0;
		current_loop = 0;
		step_timer = 0;
		interval_timer = 0;
		
		state = MODE_SMS_STATE_WAIT_PRE_TIME;
		xTimerStart(xModeSmsControlTimer, 0);
	}
	taskEXIT_CRITICAL();
}

void vModeSmsStartCameraTest(void)
{
	if (state != MODE_SMS_STATE_STOP) return;
	
	taskENTER_CRITICAL();
	{
		test_mode = mode_smsTEST_CAMERA;
		
		current_step = 0;
		current_loop = 0;
		step_timer = 0;
		interval_timer = 0;
		
		state = MODE_SMS_STATE_WAIT_PRE_TIME;
		xTimerStart(xModeSmsControlTimer, 0);
	}
	taskEXIT_CRITICAL();
}

void vModeSmsStart(void)
{
	if (state != MODE_SMS_STATE_STOP) return;
	
	taskENTER_CRITICAL();
	{
		for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
		{
			if (eep_params.mode_sms_count > 1)
			{
				int32_t steps = eep_params.mode_sms_positions[1].pos[motor] - eep_params.mode_sms_positions[0].pos[motor];
				if (eep_params.mode_sms_ramp_count > 0)
				{
					int32_t run_count = eep_params.mode_sms_count - 2 * (eep_params.mode_sms_ramp_count - 1) - 2 * eep_params.mode_sms_stall_count - 1;
					ramp_steps_x1000[motor] = steps * 1000 / (((int32_t)eep_params.mode_sms_ramp_count - 1) * (int32_t)eep_params.mode_sms_ramp_count + run_count * (int32_t)eep_params.mode_sms_ramp_count);
					run_steps[motor] = (int32_t)eep_params.mode_sms_ramp_count * ramp_steps_x1000[motor] / 1000;
				}
				else
				{
					int32_t run_count = eep_params.mode_sms_count - 2 * eep_params.mode_sms_stall_count;
					ramp_steps_x1000[motor] = 0;
					run_steps[motor] = steps / run_count;
				}                
			}
			else
			{
				ramp_steps_x1000[motor] = 0;
				run_steps[motor] = 0;
			}
		}
		
		test_mode = mode_smsTEST_NONE;
		
		current_step = 0;
		current_loop = 0;
		step_timer = 0;
		interval_timer = 0;
   
        vModeSmsCalcTime();
		
		overall_time = eep_params.mode_sms_count * eep_params.mode_sms_interval;

		state = MODE_SMS_STATE_WAKE_SM;
		xTimerStart(xModeSmsControlTimer, 0);
	}
	taskEXIT_CRITICAL();
}

void vModeSmsStop(void)
{
    taskENTER_CRITICAL();
    {
		xTimerStop(xModeSmsControlTimer, 0);
		
		for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
		{
			vSmStop(motor);
			if (eep_params.sm[motor].power_save == 0) vSmEnable(motor, 0);
		}
        vCamClear();
		test_mode = mode_smsTEST_NONE;
		state = MODE_SMS_STATE_STOP;
		finished = true;
	}
	taskEXIT_CRITICAL();
}

bool bModeSmsGetCameraTestMode(void)
{
	bool v;
	
	taskENTER_CRITICAL();
	{
		v = test_mode == mode_smsTEST_CAMERA;
	}
	taskEXIT_CRITICAL();
	
	return v;
}

bool bModeSmsGetFinished(void)
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

bool bModeSmsGetPaused(void)
{
	if (state == MODE_SMS_STATE_STOP) return false;
	
	return xTimerIsTimerActive(xModeSmsControlTimer) == pdFALSE;
}

uint8_t ucModeSmsGetState(void)
{
	uint8_t v;
	
    taskENTER_CRITICAL();
    {
		v = state;
	}
	taskEXIT_CRITICAL();
	
	return v;
}

uint32_t ulModeSmsGetCurrentCycle(void)
{
	uint32_t v;
	
	taskENTER_CRITICAL();
	{
		v = current_step;
	}
	taskEXIT_CRITICAL();
	
	return v;
}

uint32_t ulModeSmsGetRemainingCycles(void)
{
	uint32_t v;
	
	taskENTER_CRITICAL();
	{
		v = current_step;
	}
	taskEXIT_CRITICAL();
	
	return eep_params.mode_sms_count - v;
}

uint32_t ulModeSmsGetOverallCycles(void)
{
	return eep_params.mode_sms_count;
}

uint8_t ucModeSmsGetProgress(void)
{
	int16_t progress = 100 * current_step / eep_params.mode_sms_count;
	if (progress > 100) progress = 100;
	if (progress < 0) progress = 0;
	return (uint8_t)progress;
}

uint32_t ulModeSmsGetRemainingStepTime(void)
{
	uint32_t v;
	
	taskENTER_CRITICAL();
	{
		v = remaining_step_time;
	}
	taskEXIT_CRITICAL();
	
	return v;
}

uint32_t ulModeSmsGetRemainingIntervalTime(void)
{
	uint32_t v = 0;
	
	taskENTER_CRITICAL();
	{
		if ((state >= MODE_SMS_STATE_WAIT_PRE_TIME) && (eep_params.mode_sms_interval >= interval_timer))
		{
			v = eep_params.mode_sms_interval - interval_timer;
		}
	}
	taskEXIT_CRITICAL();
	
	return v;
}

uint32_t ulModeSmsGetRemainingTime(void)
{
	uint32_t v;
	
	taskENTER_CRITICAL();
	{
		v = remaining_time;
	}
	taskEXIT_CRITICAL();
	
	return v;
}

uint32_t ulModeSmsGetCurrentTime(void)
{
	uint32_t v;
	
	taskENTER_CRITICAL();
	{
		v = elapsed_time;
	}
	taskEXIT_CRITICAL();
	
	return v;
}

uint32_t ulModeSmsGetOverallTime(void)
{
	return overall_time;
}

void vModeSmsCalcTime(void)
{
    remaining_time = (eep_params.mode_sms_count - current_loop) * eep_params.mode_sms_interval - interval_timer;
	elapsed_time = current_loop * eep_params.mode_sms_interval + interval_timer;
}

static void prvModeSmsControlCallback(void *pvParameters)
{
	if (state >= MODE_SMS_STATE_WAIT_PRE_TIME && state <= MODE_SMS_STATE_WAIT_INTERVAL)
	{
		step_timer += 10;
		if (remaining_step_time > 0) remaining_step_time -= 10;
		interval_timer += 10;
	}

	switch (state)
	{
		case MODE_SMS_STATE_STOP:
			xTimerStop(xModeSmsControlTimer, 0);
			break;
		case MODE_SMS_STATE_WAKE_SM:
			{
				for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
				{
					if (eep_params.sm[motor].power_save == 0) vSmEnable(motor, 1);
					if (eep_params.sm[motor].power_save == 2) vSmEnable(motor, 2);
				}
			}
            state = MODE_SMS_STATE_GOTO_START;
			SEGGER_RTT_printf(0, "ModeSms Control State Change: GOTO_START\n");
			break;
		case MODE_SMS_STATE_GOTO_START:
			{
				for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
				{
					ucSmMove(motor, eep_params.mode_sms_positions[0].pos[motor] - lSmGetPosition(motor));
				}
			}
            state = MODE_SMS_STATE_WAIT_START;
			SEGGER_RTT_printf(0, "ModeSms Control State Change: WAIT_START\n");
		    break;
		case MODE_SMS_STATE_WAIT_START:
            if ((ucSmGetState(0) == SM_STATE_STOP) && (ucSmGetState(1) == SM_STATE_STOP) && (ucSmGetState(2) == SM_STATE_STOP))
            {
                state = MODE_SMS_STATE_WAIT_PRE_TIME;
				SEGGER_RTT_printf(0, "ModeSms Control State Change: WAIT_PRE_TIME\n");
                step_timer = 0;
				remaining_step_time = eep_params.mode_sms_pre_time;
                interval_timer = 0;
            }                
		    break;
		case MODE_SMS_STATE_WAIT_PRE_TIME:
			if (step_timer >= eep_params.mode_sms_pre_time || test_mode == mode_smsTEST_EXPOSE_NOW)
			{
				state = MODE_SMS_STATE_WAIT_FOCUS_TIME;
				SEGGER_RTT_printf(0, "ModeSms Control State Change: WAIT_FOCUS_TIME\n");
				step_timer = 0;
				remaining_step_time = eep_params.mode_sms_focus_time;
                vCamFocus();
			}
			break;
		case MODE_SMS_STATE_WAIT_FOCUS_TIME:
			if (step_timer >= eep_params.mode_sms_focus_time)
			{
				current_step++;
                vCamShutter();
                
				state = MODE_SMS_STATE_WAIT_EXPOSURE_TIME;
				SEGGER_RTT_printf(0, "ModeSms Control State Change: WAIT_EXPOSURE_TIME\n");
				step_timer = 0;
				remaining_step_time = eep_params.mode_sms_exposure_time;
			}
			break;
		case MODE_SMS_STATE_WAIT_EXPOSURE_TIME:
			if (step_timer >= eep_params.mode_sms_exposure_time)
			{
				if (test_mode == mode_smsTEST_EXPOSE_NOW)
				{
					state = MODE_SMS_STATE_STOP;
					SEGGER_RTT_printf(0, "ModeSms Control State Change: STOP\n");
				} 
				else 
				{
					state = MODE_SMS_STATE_WAIT_POST_TIME;
					SEGGER_RTT_printf(0, "ModeSms Control State Change: WAIT_POST_TIME\n");
				}
				step_timer = 0;
				remaining_step_time = eep_params.mode_sms_post_time;
				vCamClear();
			}
			break;
		case MODE_SMS_STATE_WAIT_POST_TIME:
			if (step_timer >= eep_params.mode_sms_post_time)
			{
				if (current_step >= eep_params.mode_sms_count && test_mode == mode_smsTEST_NONE)
				{
					state = MODE_SMS_STATE_GOTO_END;
					SEGGER_RTT_printf(0, "ModeSms Control State Change: GOTO_END\n");
				}
				else
				{
					state = MODE_SMS_STATE_MOVE;
					SEGGER_RTT_printf(0, "ModeSms Control State Change: MOVE\n");
				}
			}
			break;
		case MODE_SMS_STATE_MOVE:
            {
				if (test_mode == mode_smsTEST_NONE)
				{
					// calculate available time for mode_sms movement
					int32_t avail_move_time = eep_params.mode_sms_interval - (eep_params.mode_sms_pre_time + eep_params.mode_sms_post_time + eep_params.mode_sms_focus_time + eep_params.mode_sms_exposure_time);
				
					// reduce used time on ~90% and divide by 2 (for acceleration and deceleration)
					avail_move_time = avail_move_time * 90 / 200;

					// sanity check
					if (avail_move_time < 100) avail_move_time = 100;

					for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
					{
						int32_t steps = 0;
                
						if (current_step < eep_params.mode_sms_stall_count + 1)
						{
							steps = 0;
						}                
						else if (current_step < eep_params.mode_sms_stall_count + eep_params.mode_sms_ramp_count)
						{
							steps = ramp_steps_x1000[motor] * ((int32_t)current_step - (int32_t)eep_params.mode_sms_stall_count) / 1000L;
						}                
						else if (current_step < eep_params.mode_sms_count - eep_params.mode_sms_ramp_count - eep_params.mode_sms_stall_count + 1)
						{
							steps = run_steps[motor];
						}                
						else if (current_step < eep_params.mode_sms_count - eep_params.mode_sms_stall_count)
						{
							steps = ramp_steps_x1000[motor] * ((int32_t)eep_params.mode_sms_count - (int32_t)eep_params.mode_sms_stall_count - (int32_t)current_step) / 1000L;
						}                
						else
						{
							steps = 0;
						}
                
						// prevent the mode_sms to move beyond the end position
						if (eep_params.mode_sms_positions[1].pos[motor] - eep_params.mode_sms_positions[0].pos[motor] >= 0)
						{
							if (lSmGetPosition(motor) + steps > eep_params.mode_sms_positions[1].pos[motor])
							{
								steps = eep_params.mode_sms_positions[1].pos[motor] - lSmGetPosition(motor);
							}
						}
						else
						{
							if (lSmGetPosition(motor) + steps < eep_params.mode_sms_positions[1].pos[motor])
							{
								steps = eep_params.mode_sms_positions[1].pos[motor] - lSmGetPosition(motor);
							}
						}
				
						if (eep_params.sm[motor].power_save == 1) vSmEnable(motor, 1);
						if (eep_params.mode_sms_optimize_accel)
						{
							// calculate optimal acceleration
							uint16_t accel = SM_STEPS_TO_MRAD(labs(steps)) * 1000000UL / (avail_move_time * avail_move_time);
				
							accel = utilsMIN(accel, SM_STEPS_TO_MRAD(eep_params.sm[motor].accel_steps));
							accel = utilsMAX(accel, SM_STEPS_TO_MRAD(SM_SPR/16));
							uint16_t max_speed = accel * avail_move_time / 1000;
								
							ucSmMoveEx(motor, steps, max_speed, accel, accel);
							//ucSmMoveEx(motor, steps, SM_STEPS_TO_MRAD(eep_params.sm[motor].speed_max_steps), accel, accel);
						}
						else
						{
							ucSmMove(motor, steps);
						}
					}
					state = MODE_SMS_STATE_WAIT_MOVE;
					SEGGER_RTT_printf(0, "ModeSms Control State Change: WAIT_MOVE\n");
				}
				else
				{
					state = MODE_SMS_STATE_WAIT_INTERVAL;
					SEGGER_RTT_printf(0, "ModeSms Control State Change: WAIT_INTERVAL\n");
				}
            }
			break;
		case MODE_SMS_STATE_WAIT_MOVE:
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
					state = MODE_SMS_STATE_WAIT_INTERVAL;
					SEGGER_RTT_printf(0, "ModeSms Control State Change: WAIT_INTERVAL\n");
				}
			}
			break;
		case MODE_SMS_STATE_WAIT_INTERVAL:
			if (interval_timer >= eep_params.mode_sms_interval)
			{
                if (interval_timer > eep_params.mode_sms_interval)
                {
                    eep_params.mode_sms_interval = interval_timer + 50;
                }                    
				state = MODE_SMS_STATE_WAIT_PRE_TIME;
				SEGGER_RTT_printf(0, "ModeSms Control State Change: WAIT_PRE_TIME\n");
				step_timer = 0;
				remaining_step_time = eep_params.mode_sms_pre_time;
				
				interval_timer = 0;
				current_loop++;
			}
			break;
		case MODE_SMS_STATE_GOTO_END:
			{
				for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
				{
					if (eep_params.sm[motor].power_save == 1) vSmEnable(motor, 1);
					ucSmMove(motor, eep_params.mode_sms_positions[1].pos[motor] - lSmGetPosition(motor));
				}
			}
			state = MODE_SMS_STATE_WAIT_END;
			SEGGER_RTT_printf(0, "ModeSms Control State Change: WAIT_END\n");
			break;
		case MODE_SMS_STATE_WAIT_END:
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
					state = MODE_SMS_STATE_SLEEP_SM;
					SEGGER_RTT_printf(0, "ModeSms Control State Change: SLEEP_SM\n");
					step_timer = 0;
					remaining_step_time = 0;
					interval_timer = 0;
					current_loop++;
				}
			}
			break;
		case MODE_SMS_STATE_SLEEP_SM:
			{
				for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
				{
					if (eep_params.sm[motor].power_save == 1) vSmEnable(motor, 0);
				}
			}
			state = MODE_SMS_STATE_STOP;
			finished = true;
			SEGGER_RTT_printf(0, "ModeSms Control State Change: STOP\n");
			break;
	}

	vModeSmsCalcTime();
}
