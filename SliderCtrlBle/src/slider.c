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
#include "slider.h"
#include "utils.h"

#define sliderBLE_TX_TIMER_RATE      (200 / portTICK_RATE_MS)
#define sliderCONTROL_TIMER_RATE     (10 / portTICK_RATE_MS)

static uint8_t state = SLIDER_STATE_STOP;
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

static void prvSliderControlCallback(void *pvParameters);

void vSliderInit(void)
{
#if !defined(DEBUG)
	for (uint8_t f = 0; f < SLIDER_MAX_KEY_FRAMES; f++)
	{
		for (uint8_t m = 0; m < SM_MOTORS_USED; m++)
		{
			eep_params.slider_positions[f].pos[m] = 0;
		}
	}
#endif

	xTimerHandle xSliderControlTimer = xTimerCreate(
		(const char * const) "SliderCtrlTmr",
		sliderCONTROL_TIMER_RATE,
		pdTRUE,
		NULL,
		prvSliderControlCallback);
	configASSERT(xSliderControlTimer);
	xTimerStart(xSliderControlTimer, 0);
}

void vSliderSetParams(slider_setup_t *params)
{
	eep_params.slider_pre_time = params->pre_time;
	eep_params.slider_focus_time = params->focus_time;
	eep_params.slider_exposure_time = params->exposure_time;
	eep_params.slider_post_time = params->post_time;

    vSliderUpdateIntervalToMinimum();
}

void vSliderGetParams(slider_setup_t *params)
{
	params->pre_time = eep_params.slider_pre_time;
	params->focus_time = eep_params.slider_focus_time;
	params->exposure_time = eep_params.slider_exposure_time;
	params->post_time = eep_params.slider_post_time;
}

void vSliderSetStartEnd(int32_t start, int32_t end)
{
    eep_params.slider_positions[0].pos[0] = start;
    eep_params.slider_positions[1].pos[0] = end;

#if DEBUG
    vEepSave();
#endif
}

void vSliderGetStartEnd(int32_t *start, int32_t *end)
{
    *start = eep_params.slider_positions[0].pos[0];
    *end = eep_params.slider_positions[1].pos[0];
}

void vSliderSetInterval(uint32_t interval, uint32_t count, uint16_t ramp_count, uint16_t stall_count)
{
    eep_params.slider_interval = interval;
    eep_params.slider_count = count;
    eep_params.slider_ramp_count = ramp_count;
    eep_params.slider_stall_count = stall_count;
    vEepSave();
}

void vSliderGetInterval(uint32_t *interval, uint32_t *count, uint16_t *ramp_count, uint16_t *stall_count)
{
    *interval = eep_params.slider_interval;
    *count = eep_params.slider_count;
    *ramp_count = eep_params.slider_ramp_count;
    *stall_count = eep_params.slider_stall_count;
}

void vSliderUpdateIntervalToMinimum(void)
{
    eep_params.slider_interval = utilsMAX(eep_params.slider_interval, ulSliderGetMinimumInterval(eep_params.slider_pre_time, eep_params.slider_focus_time, eep_params.slider_exposure_time, eep_params.slider_post_time));
    vEepSave();
}

uint32_t ulSliderGetMinimumInterval(uint32_t pre_time, uint32_t focus_time, uint32_t exposure_time, uint32_t post_time)
{
    return pre_time + focus_time + exposure_time + post_time + 500;
}

void vSliderStart()
{
	taskENTER_CRITICAL();
	{
		for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
		{
			if (eep_params.slider_count > 1)
			{
				int32_t steps = eep_params.slider_positions[1].pos[motor] - eep_params.slider_positions[0].pos[motor];
				if (eep_params.slider_ramp_count > 0)
				{
					int32_t run_count = eep_params.slider_count - 2 * (eep_params.slider_ramp_count - 1) - 2 * eep_params.slider_stall_count - 1;
					ramp_steps_x1000[motor] = steps * 1000 / (((int32_t)eep_params.slider_ramp_count - 1) * (int32_t)eep_params.slider_ramp_count + run_count * (int32_t)eep_params.slider_ramp_count);
					run_steps[motor] = (int32_t)eep_params.slider_ramp_count * ramp_steps_x1000[motor] / 1000;
				}
				else
				{
					int32_t run_count = eep_params.slider_count - 2 * eep_params.slider_stall_count;
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
		
		current_step = 0;
		current_loop = 0;
		step_timer = 0;
		interval_timer = 0;
   
        vSliderCalcTime();
		
		overall_time = eep_params.slider_count * eep_params.slider_interval;

		state = SLIDER_STATE_WAKE_SM;
	}
	taskEXIT_CRITICAL();
}

void vSliderStop(void)
{
    taskENTER_CRITICAL();
    {
		for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
		{
			vSmStop(motor);
			vSmEnable(motor, 0);
		}
        vCamClear();
		state = SLIDER_STATE_STOP;
	}
	taskEXIT_CRITICAL();
}

uint8_t ucSliderGetState(void)
{
	uint8_t v;
	
    taskENTER_CRITICAL();
    {
		v = state;
	}
	taskEXIT_CRITICAL();
	
	return v;
}

uint32_t ulSliderGetCurrentCycle(void)
{
	uint32_t v;
	
	taskENTER_CRITICAL();
	{
		v = current_step;
	}
	taskEXIT_CRITICAL();
	
	return v;
}

uint32_t ulSliderGetRemainingCycles(void)
{
	uint32_t v;
	
	taskENTER_CRITICAL();
	{
		v = current_step;
	}
	taskEXIT_CRITICAL();
	
	return eep_params.slider_count - v;
}

uint32_t ulSliderGetOverallCycles(void)
{
	return eep_params.slider_count;
}

uint8_t ucSliderGetProgress(void)
{
	int16_t progress = 100 * current_step / eep_params.slider_count;
	if (progress > 100) progress = 100;
	if (progress < 0) progress = 0;
	return (uint8_t)progress;
}

uint32_t ulSliderGetRemainingStepTime(void)
{
	uint32_t v;
	
	taskENTER_CRITICAL();
	{
		v = remaining_step_time;
	}
	taskEXIT_CRITICAL();
	
	return v;
}

uint32_t ulSliderGetRemainingIntervalTime(void)
{
	uint32_t v = 0;
	
	taskENTER_CRITICAL();
	{
		if ((state >= SLIDER_STATE_WAIT_PRE_TIME) && (eep_params.slider_interval >= interval_timer))
		{
			v = eep_params.slider_interval - interval_timer;
		}
	}
	taskEXIT_CRITICAL();
	
	return v;
}

uint32_t ulSliderGetRemainingTime(void)
{
	uint32_t v;
	
	taskENTER_CRITICAL();
	{
		v = remaining_time;
	}
	taskEXIT_CRITICAL();
	
	return v;
}

uint32_t ulSliderGetCurrentTime(void)
{
	uint32_t v;
	
	taskENTER_CRITICAL();
	{
		v = elapsed_time;
	}
	taskEXIT_CRITICAL();
	
	return v;
}

uint32_t ulSliderGetOverallTime(void)
{
	return overall_time;
}

void vSliderCalcTime(void)
{
    remaining_time = (eep_params.slider_count - current_loop) * eep_params.slider_interval - interval_timer;
	elapsed_time = current_loop * eep_params.slider_interval + interval_timer;
}

static void prvSliderControlCallback(void *pvParameters)
{
	if (state >= SLIDER_STATE_WAIT_PRE_TIME && state <= SLIDER_STATE_WAIT_INTERVAL)
	{
		step_timer += 10;
		if (remaining_step_time > 0) remaining_step_time -= 10;
		interval_timer += 10;
	}

	switch (state)
	{
		case SLIDER_STATE_STOP:
			break;
		case SLIDER_STATE_WAKE_SM:
			{
				for (uint8_t m = 0; m < SM_MOTORS_USED; m++)
				{
					if (eep_params.sm[m].power_save == 0) vSmEnable(m, 1);
				}
			}
            state = SLIDER_STATE_GOTO_START;
			break;
		case SLIDER_STATE_GOTO_START:
			{
				for (uint8_t m = 0; m < SM_MOTORS_USED; m++)
				{
					ucSmMove(m, eep_params.slider_positions[0].pos[m] - lSmGetPosition(m));
				}
			}
            state = SLIDER_STATE_WAIT_START;
		    break;
		case SLIDER_STATE_WAIT_START:
            if ((ucSmGetState(0) == SM_STATE_STOP) && (ucSmGetState(1) == SM_STATE_STOP) && (ucSmGetState(2) == SM_STATE_STOP))
            {
                state = SLIDER_STATE_WAIT_PRE_TIME;
                step_timer = 0;
				remaining_step_time = eep_params.slider_pre_time;
                interval_timer = 0;
            }                
		    break;
		case SLIDER_STATE_WAIT_PRE_TIME:
			if (step_timer >= eep_params.slider_pre_time)
			{
				state = SLIDER_STATE_WAIT_FOCUS_TIME;
				step_timer = 0;
				remaining_step_time = eep_params.slider_focus_time;
                vCamFocus();
			}
			break;
		case SLIDER_STATE_WAIT_FOCUS_TIME:
			if (step_timer >= eep_params.slider_focus_time)
			{
				current_step++;
                vCamShutter();
                
				state = SLIDER_STATE_WAIT_EXPOSURE_TIME;
				step_timer = 0;
				remaining_step_time = eep_params.slider_exposure_time;
			}
			break;
		case SLIDER_STATE_WAIT_EXPOSURE_TIME:
			if (step_timer >= eep_params.slider_exposure_time)
			{
				state = SLIDER_STATE_WAIT_POST_TIME;
				step_timer = 0;
				remaining_step_time = eep_params.slider_post_time;
				vCamClear();
			}
			break;
		case SLIDER_STATE_WAIT_POST_TIME:
			if (step_timer >= eep_params.slider_post_time)
			{
				if (current_step >= eep_params.slider_count)
				{
					state = SLIDER_STATE_GOTO_END;
				}
				else
				{
					state = SLIDER_STATE_MOVE;
				}
			}
			break;
		case SLIDER_STATE_MOVE:
            {
				for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
				{
					int32_t steps = 0;
                
					if (current_step < eep_params.slider_stall_count + 1)
					{
						steps = 0;
					}                
					else if (current_step < eep_params.slider_stall_count + eep_params.slider_ramp_count)
					{
						steps = ramp_steps_x1000[motor] * ((int32_t)current_step - (int32_t)eep_params.slider_stall_count) / 1000L;
					}                
					else if (current_step < eep_params.slider_count - eep_params.slider_ramp_count - eep_params.slider_stall_count + 1)
					{
						steps = run_steps[motor];
					}                
					else if (current_step < eep_params.slider_count - eep_params.slider_stall_count)
					{
						steps = ramp_steps_x1000[motor] * ((int32_t)eep_params.slider_count - (int32_t)eep_params.slider_stall_count - (int32_t)current_step) / 1000L;
					}                
					else
					{
						steps = 0;
					}
                
					// prevent the slider to move beyond the end position
					if (eep_params.slider_positions[1].pos[motor] - eep_params.slider_positions[0].pos[motor] >= 0)
					{
						if (lSmGetPosition(motor) + steps > eep_params.slider_positions[1].pos[motor])
						{
							steps = eep_params.slider_positions[1].pos[motor] - lSmGetPosition(motor);
						}
					}
					else
					{
						if (lSmGetPosition(motor) + steps < eep_params.slider_positions[1].pos[motor])
						{
							steps = eep_params.slider_positions[1].pos[motor] - lSmGetPosition(motor);
						}
					}
				
					if (eep_params.slider_optimize_accel)
					{
						// calculate available time for slider movement
						int32_t avail_move_time = eep_params.slider_interval - (eep_params.slider_pre_time + eep_params.slider_post_time + eep_params.slider_focus_time + eep_params.slider_exposure_time);
				
						// reduce used time on ~90% and divide by 2 (for acceleration and deceleration)
						avail_move_time = avail_move_time * 90 / 200;

						// sanity check
						if (avail_move_time < 100) avail_move_time = 100;
				
						// calculate optimal acceleration
						uint16_t accel = SM_STEPS_TO_MRAD(labs(steps)) * 1000000UL / (avail_move_time * avail_move_time);
				
						accel = utilsMIN(accel, SM_STEPS_TO_MRAD(eep_params.sm[motor].accel_steps));
						accel = utilsMAX(accel, SM_STEPS_TO_MRAD(SM_SPR/4));
								
						ucSmMoveEx(motor, steps, SM_STEPS_TO_MRAD(eep_params.sm[motor].speed_max_steps), accel, accel);
					}
					else
					{
						ucSmMove(motor, steps);
					}
				}
            }
            state = SLIDER_STATE_WAIT_MOVE;
			break;
		case SLIDER_STATE_WAIT_MOVE:
			if ((ucSmGetState(0) == SM_STATE_STOP) && (ucSmGetState(1) == SM_STATE_STOP) && (ucSmGetState(2) == SM_STATE_STOP))
			{
					state = SLIDER_STATE_WAIT_INTERVAL;
			}
			break;
		case SLIDER_STATE_WAIT_INTERVAL:
			if (interval_timer >= eep_params.slider_interval)
			{
                if (interval_timer > eep_params.slider_interval)
                {
                    eep_params.slider_interval = interval_timer + 50;
                }                    
				state = SLIDER_STATE_WAIT_PRE_TIME;
				step_timer = 0;
				remaining_step_time = eep_params.slider_pre_time;
				
				interval_timer = 0;
				current_loop++;
			}
			break;
		case SLIDER_STATE_GOTO_END:
			{
				for (uint8_t m = 0; m < SM_MOTORS_USED; m++)
				{
					ucSmMove(m, eep_params.slider_positions[1].pos[m] - lSmGetPosition(m));
				}
			}
			state = SLIDER_STATE_WAIT_END;
			break;
		case SLIDER_STATE_WAIT_END:
			if ((ucSmGetState(0) == SM_STATE_STOP) && (ucSmGetState(1) == SM_STATE_STOP) && (ucSmGetState(2) == SM_STATE_STOP))
			{
				state = SLIDER_STATE_SLEEP_SM;
                step_timer = 0;
                remaining_step_time = 0;
				interval_timer = 0;
				current_loop++;
			}
			break;
		case SLIDER_STATE_SLEEP_SM:
			{
				for (uint8_t m = 0; m < SM_MOTORS_USED; m++)
				{
					if (eep_params.sm[m].power_save == 0) vSmEnable(m, 0);
				}
			}
			state = SLIDER_STATE_STOP;
			break;
	}

	vSliderCalcTime();
}
