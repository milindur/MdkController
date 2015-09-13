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
#include "mode_pano.h"
#include "utils.h"

#define mode_panoCONTROL_TIMER_RATE     (10 / portTICK_RATE_MS)

static uint8_t state = MODE_PANO_STATE_STOP;
static bool finished = false;
static uint32_t current_step;
static uint32_t current_row;
static uint32_t max_rows;
static uint32_t current_col;
static uint32_t max_cols;
static int32_t steps[SM_MOTORS_USED];
static uint32_t step_timer;
static xTimerHandle xModePanoControlTimer;

static void prvModePanoControlCallback(void *pvParameters);

void vModePanoInit(void)
{
	for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
	{
		eep_params.mode_pano_position_start.pos[motor] = 0;
		eep_params.mode_pano_position_stop.pos[motor] = 0;
		eep_params.mode_pano_position_ref_start.pos[motor] = 0;
		eep_params.mode_pano_position_ref_stop.pos[motor] = 0;
	}

	xModePanoControlTimer = xTimerCreate(
		(const char * const) "ModePanoCtrlTmr",
		mode_panoCONTROL_TIMER_RATE,
		pdTRUE,
		NULL,
		prvModePanoControlCallback);
	configASSERT(xModePanoControlTimer);
}

void vModePanoPause(void)
{
	if (state == MODE_PANO_STATE_STOP) return;
	if (bModePanoIsPaused()) return;
	
	xTimerStop(xModePanoControlTimer, 0);
}

void vModePanoResume(void)
{
	if (state == MODE_PANO_STATE_STOP) return;
	if (!bModePanoIsPaused()) return;
	
	xTimerStart(xModePanoControlTimer, 0);
}

void vModePanoStart(void)
{
	if (state != MODE_PANO_STATE_STOP) return;
	
	taskENTER_CRITICAL();
	{
		int32_t pan_steps_per_col = abs(eep_params.mode_pano_position_ref_stop.pos[1] - eep_params.mode_pano_position_ref_start.pos[1]) * (100 - 34) / 100;
		int32_t tilt_steps_per_row = abs(eep_params.mode_pano_position_ref_stop.pos[2] - eep_params.mode_pano_position_ref_start.pos[2]) * (100 - 34) / 100;

		if (eep_params.mode_pano_position_stop.pos[1] < eep_params.mode_pano_position_start.pos[1]) pan_steps_per_col *= -1;
		if (eep_params.mode_pano_position_stop.pos[2] < eep_params.mode_pano_position_start.pos[2]) tilt_steps_per_row *= -1;

		max_cols = abs((eep_params.mode_pano_position_stop.pos[1] - eep_params.mode_pano_position_start.pos[1]) / pan_steps_per_col) + 1;
		max_rows = abs((eep_params.mode_pano_position_stop.pos[2] - eep_params.mode_pano_position_start.pos[2]) / tilt_steps_per_row) + 1;
		
		if (eep_params.mode_pano_position_start.pos[1] + (max_cols - 1) * pan_steps_per_col < eep_params.mode_pano_position_stop.pos[1])
		{
			max_cols++;
		}
		if (eep_params.mode_pano_position_start.pos[2] + (max_rows - 1) * tilt_steps_per_row < eep_params.mode_pano_position_stop.pos[2])
		{
			max_rows++;
		}
		
		pan_steps_per_col = (int32_t)(eep_params.mode_pano_position_stop.pos[1] - eep_params.mode_pano_position_start.pos[1]) / (int32_t)max_cols;
		tilt_steps_per_row = (int32_t)(eep_params.mode_pano_position_stop.pos[2] - eep_params.mode_pano_position_start.pos[2]) / (int32_t)max_rows;
		
		steps[1] = pan_steps_per_col;
		steps[2] = tilt_steps_per_row;
		
        current_step = 0;
		current_row = 0;
		current_col = 0;
		step_timer = 0;

		state = MODE_PANO_STATE_WAKE_SM;
		xTimerStart(xModePanoControlTimer, 0);
	}
	taskEXIT_CRITICAL();
}

void vModePanoStop(void)
{
	taskENTER_CRITICAL();
	{
		xTimerStop(xModePanoControlTimer, 0);
		
		for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
		{
			vSmStop(motor);
			if (eep_params.sm[motor].power_save == 0) vSmEnable(motor, 0);
		}
		vCamClear();
		state = MODE_PANO_STATE_STOP;
		finished = true;
	}
	taskEXIT_CRITICAL();
}


bool bModePanoIsFinished(void)
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

bool bModePanoIsPaused(void)
{
	if (state == MODE_PANO_STATE_STOP) return false;
	
	return xTimerIsTimerActive(xModePanoControlTimer) == pdFALSE;
}

bool bModePanoIsRunning(void)
{
	return state != MODE_PANO_STATE_STOP;
}

uint32_t ulModePanoGetCurrentCycle(void)
{
    uint32_t v;
    
    taskENTER_CRITICAL();
    {
        v = current_step;
    }
    taskEXIT_CRITICAL();
    
    return v;
}

uint32_t ulModePanoGetRemainingCycles(void)
{
    uint32_t v;
    
    taskENTER_CRITICAL();
    {
        v = current_step;
    }
    taskEXIT_CRITICAL();
    
    return (max_rows + 1) * (max_cols + 1) - v;
}

uint32_t ulModePanoGetOverallCycles(void)
{
    return (max_rows + 1) * (max_cols + 1);
}

uint32_t ulModePanoGetCurrentRow(void)
{
	return current_col;
}

uint32_t ulModePanoGetOverallRows(void)
{
	return max_rows + 1;
}

uint32_t ulModePanoGetCurrentCol(void)
{
	return current_col;
}

uint32_t ulModePanoGetOverallCols(void)
{
	return max_cols + 1;
}

uint8_t ucModePanoGetProgress(void)
{
	int16_t progress = 100 * current_step / ((max_rows + 1) * (max_cols + 1));
	if (progress > 100) progress = 100;
	if (progress < 0) progress = 0;
	return (uint8_t)progress;
}

uint8_t ucModePanoGetState(void)
{
	uint8_t v;
	
	taskENTER_CRITICAL();
	{
		v = state;
	}
	taskEXIT_CRITICAL();
	
	return v;
}


static void prvModePanoControlCallback(void *pvParameters)
{
	if (state >= MODE_PANO_STATE_WAIT_PRE_TIME && state <= MODE_PANO_STATE_WAIT_MOVE)
	{
		step_timer += 10;
	}

	switch (state)
	{
		case MODE_PANO_STATE_STOP:
			xTimerStop(xModePanoControlTimer, 0);
			break;
		case MODE_PANO_STATE_WAKE_SM:
			{
				for (uint8_t motor = 1; motor <= 2; motor++)
				{
					if (eep_params.sm[motor].power_save == 0) vSmEnable(motor, 1);
					if (eep_params.sm[motor].power_save == 2) vSmEnable(motor, 2);
				}
			}
			state = MODE_PANO_STATE_GOTO_START;
			SEGGER_RTT_printf(0, "ModePano Control State Change: GOTO_START\n");
			break;
		case MODE_PANO_STATE_GOTO_START:
			{
				for (uint8_t motor = 1; motor <= 2; motor++)
				{
					ucSmMove(motor, eep_params.mode_pano_position_start.pos[motor] - lSmGetPosition(motor));
				}
			}
			state = MODE_SMS_STATE_WAIT_START;
			SEGGER_RTT_printf(0, "ModePano Control State Change: WAIT_START\n");
			break;
		case MODE_PANO_STATE_WAIT_START:
			if ((ucSmGetState(0) == SM_STATE_STOP) && (ucSmGetState(1) == SM_STATE_STOP) && (ucSmGetState(2) == SM_STATE_STOP))
			{
				state = MODE_PANO_STATE_WAIT_PRE_TIME;
				SEGGER_RTT_printf(0, "ModePano Control State Change: WAIT_PRE_TIME\n");
				step_timer = 0;
			}
			break;
		case MODE_PANO_STATE_WAIT_PRE_TIME:
			if (step_timer >= eep_params.mode_sms_pre_time)
			{
				state = MODE_SMS_STATE_WAIT_FOCUS_TIME;
				SEGGER_RTT_printf(0, "ModePano Control State Change: WAIT_FOCUS_TIME\n");
				step_timer = 0;
				vCamFocus();
			}
			break;
		case MODE_PANO_STATE_WAIT_FOCUS_TIME:
			if (step_timer >= eep_params.mode_sms_focus_time)
			{
                current_step++;
				vCamShutter();
			
				state = MODE_PANO_STATE_WAIT_EXPOSURE_TIME;
				SEGGER_RTT_printf(0, "ModePano Control State Change: WAIT_EXPOSURE_TIME\n");
				step_timer = 0;
			}
			break;
		case MODE_PANO_STATE_WAIT_EXPOSURE_TIME:
			if (step_timer >= eep_params.mode_sms_exposure_time)
			{
				state = MODE_PANO_STATE_WAIT_POST_TIME;
				SEGGER_RTT_printf(0, "ModePano Control State Change: WAIT_POST_TIME\n");
				step_timer = 0;
				vCamClear();
			}
			break;
		case MODE_PANO_STATE_WAIT_POST_TIME:
			if (step_timer >= eep_params.mode_sms_post_time)
			{
				if ((current_row % 2) == 0)
				{
					if (current_col < max_cols)
					{
						current_col++;
					}
					else
					{
						current_row++;
					}
				}
				else
				{
					if (current_col > 0)
					{
						current_col--;
					}
					else
					{
						current_row++;
					}
				}
				
				if (current_row > max_rows)
				{
					state = MODE_PANO_STATE_SLEEP_SM;
					SEGGER_RTT_printf(0, "ModePano Control State Change: SLEEP_SM\n");
				}
				else
				{
					state = MODE_PANO_STATE_MOVE;
					SEGGER_RTT_printf(0, "ModePano Control State Change: MOVE\n");
				}
			}
			break;
		case MODE_PANO_STATE_MOVE:
			{
				int32_t pan_steps = eep_params.mode_pano_position_start.pos[1] 
					+ current_col * steps[1] 
					- lSmGetPosition(1);
				int32_t tilt_steps = eep_params.mode_pano_position_start.pos[2]
					+ current_row * steps[2]
					- lSmGetPosition(2);

				ucSmMove(1, pan_steps);
				ucSmMove(2, tilt_steps);
				
				state = MODE_PANO_STATE_WAIT_MOVE;
				SEGGER_RTT_printf(0, "ModePano Control State Change: WAIT_MOVE\n");
			}
			break;
		case MODE_PANO_STATE_WAIT_MOVE:
			{
				bool all_stoped = true;
				for (uint8_t motor = 1; motor <= 2; motor++)
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
					state = MODE_SMS_STATE_WAIT_PRE_TIME;
					SEGGER_RTT_printf(0, "ModePano Control State Change: WAIT_PRE_TIME\n");
				}
			}
			break;
		case MODE_PANO_STATE_SLEEP_SM:
			{
				for (uint8_t motor = 1; motor <= 2; motor++)
				{
					if (eep_params.sm[motor].power_save == 1) vSmEnable(motor, 0);
				}
			}
			state = MODE_PANO_STATE_STOP;
			finished = true;
			SEGGER_RTT_printf(0, "ModePano Control State Change: STOP\n");
			break;
	}
}
