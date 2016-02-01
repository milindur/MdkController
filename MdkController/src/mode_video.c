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
static uint8_t move_state[SM_MOTORS_USED];
static bool finished = false;
static bool start_to_end = true;
static uint32_t step_timer[SM_MOTORS_USED];
static uint32_t interval_timer;
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
        overall_time = 0;

        for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
        {
            if (eep_params.mode_sms_accel_count[motor] == 0)
            {
                eep_params.mode_sms_accel_count[motor] = 1;
            }
            if (eep_params.mode_sms_decel_count[motor] == 0)
            {
                eep_params.mode_sms_decel_count[motor] = 1;
            }
            
            move_state[motor] = MODE_VIDEO_STATE_MOVE_WAIT_LEAD_IN;
            step_timer[motor] = 0;

            overall_time = utilsMAX(overall_time, eep_params.mode_sms_leadin_count[motor] + eep_params.mode_video_duration[motor] + eep_params.mode_sms_leadout_count[motor]);
        }

        vModeVideoCalcTime();
        
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
    int16_t progress = 100 * elapsed_time / overall_time;
    if (progress > 100) progress = 100;
    if (progress < 0) progress = 0;
    return (uint8_t)progress;
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
    remaining_time = overall_time - interval_timer;
    elapsed_time = interval_timer;
}

static void prvModeVideoControlCallback(void *pvParameters)
{
    if (state == MODE_VIDEO_STATE_MOVE)
    {
        for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
        {
            step_timer[motor] += 10;
        }

        interval_timer += 10;
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
                for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
                {
                    move_state[motor] = MODE_VIDEO_STATE_MOVE_WAIT_LEAD_IN;
                    step_timer[motor] = 0;
                }
                interval_timer = 0;
                start_to_end = true;
            }                
            break;
        case MODE_VIDEO_STATE_MOVE:
            {
                for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
                {
                    switch (move_state[motor])
                    {
                        case MODE_VIDEO_STATE_MOVE_WAIT_LEAD_IN:
                            if (step_timer[motor] >= eep_params.mode_sms_leadin_count[motor])
                            {
                                move_state[motor] = MODE_VIDEO_STATE_MOVE_RUN;
                            }
                            break;
                        case MODE_VIDEO_STATE_MOVE_RUN:
                            {
                                // calculate available time for video movement
                                int32_t move_time = (int32_t)eep_params.mode_video_duration[motor];
                                int32_t ramp_time = (int32_t)eep_params.mode_sms_accel_count[motor] + (int32_t)eep_params.mode_sms_decel_count[motor];
                                int32_t run_time = move_time - ramp_time;
                    
                                // sanity checks
                                if (move_time < 100) move_time = 100;
                                if (run_time < 100) run_time = 100;

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
                    
                                int32_t max_speed_steps = (int32_t)((int64_t)2000 * (int64_t)labs(steps) / (int64_t)(2 * run_time + ramp_time));
                                int32_t accel_steps = (int32_t)((int64_t)max_speed_steps * (int64_t)1000 / (int64_t)eep_params.mode_sms_accel_count[motor]);
                                int32_t decel_steps = (int32_t)((int64_t)max_speed_steps * (int64_t)1000 / (int64_t)eep_params.mode_sms_decel_count[motor]);

                                max_speed_steps = utilsMIN(max_speed_steps, eep_params.sm[motor].speed_max_steps);
                                max_speed_steps = utilsMAX(max_speed_steps, SM_SPR/16);
                                accel_steps = utilsMIN(accel_steps, eep_params.sm[motor].accel_steps);
                                accel_steps = utilsMAX(accel_steps, SM_SPR/16);
                                decel_steps = utilsMIN(decel_steps, eep_params.sm[motor].decel_steps);
                                decel_steps = utilsMAX(decel_steps, SM_SPR/16);

                                uint16_t max_speed = SM_STEPS_TO_MRAD(max_speed_steps);
                                uint16_t accel = SM_STEPS_TO_MRAD(accel_steps);
                                uint16_t decel = SM_STEPS_TO_MRAD(decel_steps);
                            
                                ucSmMoveEx(motor, steps, max_speed, accel, decel);

                                move_state[motor] = MODE_VIDEO_STATE_MOVE_WAIT_RUN;
                            }
                            break;
                        case MODE_VIDEO_STATE_MOVE_WAIT_RUN:
                            if (ucSmGetState(motor) == SM_STATE_STOP)
                            {
                                move_state[motor] = MODE_VIDEO_STATE_MOVE_WAIT_LEAD_OUT;
                                step_timer[motor] = 0;
                            }
                            break;
                        case MODE_VIDEO_STATE_MOVE_WAIT_LEAD_OUT:
                            if (step_timer[motor] >= eep_params.mode_sms_leadout_count[motor])
                            {
                                move_state[motor] = MODE_VIDEO_STATE_MOVE_DONE;
                            }
                            break;
                        case MODE_VIDEO_STATE_MOVE_DONE:
                            break;
                    }
                }
                
                bool all_done = true;
                for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
                {
                    if (move_state[motor] != MODE_VIDEO_STATE_MOVE_DONE)
                    {
                        all_done = false;
                        continue;
                    }
                }
                if (all_done)
                {
                    state = MODE_VIDEO_STATE_WAIT_MOVE;
                    SEGGER_RTT_printf(0, "ModeVideo Control State Change: WAIT_MOVE\n");
                }
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
                        interval_timer = 0;
                        for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
                        {
                            move_state[motor] = MODE_VIDEO_STATE_MOVE_WAIT_LEAD_IN;
                            step_timer[motor] = 0;
                        }
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
