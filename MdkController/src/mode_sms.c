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

#define mode_smsTEST_NONE               0
#define mode_smsTEST_CAMERA             1
#define mode_smsTEST_EXPOSE_NOW         2

#define mode_smsCONTROL_TIMER_RATE      (10 / portTICK_RATE_MS)

#define mode_smsCOUNT_SUM_TOO_HIGH(motor)   ((eep_params.mode_sms_leadin_count[motor] + eep_params.mode_sms_accel_count[motor] + eep_params.mode_sms_decel_count[motor] + eep_params.mode_sms_leadout_count[motor]) > (eep_params.mode_sms_count - 1))

static uint8_t state = MODE_SMS_STATE_STOP;
static bool finished = false;
static uint8_t test_mode = mode_smsTEST_NONE;
static uint32_t current_step;
static uint32_t current_loop;
static int32_t accel_steps_x1000[SM_MOTORS_USED];
static int32_t decel_steps_x1000[SM_MOTORS_USED];
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

void vModeSmsUpdateIntervalToMinimum(void)
{
    eep_params.mode_sms_interval = utilsMAX(eep_params.mode_sms_interval, ulModeSmsGetMinimumInterval(eep_params.mode_sms_pre_time, eep_params.mode_sms_focus_time, eep_params.mode_sms_exposure_time, eep_params.mode_sms_post_time));
}

uint32_t ulModeSmsGetMinimumInterval(uint32_t pre_time, uint32_t focus_time, uint32_t exposure_time, uint32_t post_time)
{
    return pre_time + focus_time + exposure_time + post_time + 500;
}

void vModeSmsPause(void)
{
    if (state == MODE_SMS_STATE_STOP) return;
    if (bModeSmsIsPaused()) return;
    
    xTimerStop(xModeSmsControlTimer, 0);
}

void vModeSmsResume(void)
{
    if (state == MODE_SMS_STATE_STOP) return;
    if (!bModeSmsIsPaused()) return;
    
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

void vModeSmsStart(uint8_t option)
{
    if (state != MODE_SMS_STATE_STOP) return;
    
    taskENTER_CRITICAL();
    {
		eep_params.mode_sms_use_slider_as_shutter = 0;
		if (option & MODE_SMS_OPTION_SHUTTER)
		{
			eep_params.mode_sms_use_slider_as_shutter = 1;
		}

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
            
            while (true)
            {
                if (mode_smsCOUNT_SUM_TOO_HIGH(motor) && eep_params.mode_sms_leadin_count[motor] > 0)
                {
                    eep_params.mode_sms_leadin_count[motor]--;
                }
                if (mode_smsCOUNT_SUM_TOO_HIGH(motor) && eep_params.mode_sms_leadout_count[motor] > 0)
                {
                    eep_params.mode_sms_leadout_count[motor]--;
                }
                if (mode_smsCOUNT_SUM_TOO_HIGH(motor) && eep_params.mode_sms_accel_count[motor] > 1)
                {
                    eep_params.mode_sms_accel_count[motor]--;
                }
                if (mode_smsCOUNT_SUM_TOO_HIGH(motor) && eep_params.mode_sms_decel_count[motor] > 1)
                {
                    eep_params.mode_sms_decel_count[motor]--;
                }
                if (!mode_smsCOUNT_SUM_TOO_HIGH(motor))
                {
                    break;
                }
            }

            int32_t steps = eep_params.mode_sms_positions[1].pos[motor] - eep_params.mode_sms_positions[0].pos[motor];

            //int32_t img_count = (int32_t)eep_params.mode_sms_count;
            int32_t step_count = (int32_t)eep_params.mode_sms_count - 1;
            int32_t ramp_count = (int32_t)eep_params.mode_sms_accel_count[motor] + (int32_t)eep_params.mode_sms_decel_count[motor];
            int32_t lead_count = (int32_t)eep_params.mode_sms_leadin_count[motor] + (int32_t)eep_params.mode_sms_leadout_count[motor];
            int32_t move_count = step_count - lead_count;
            int32_t run_count = step_count - lead_count - ramp_count;

            if (move_count > 0 && steps != 0)
            {
                run_steps[motor] = 2 * steps / (2 * run_count + ramp_count);
                accel_steps_x1000[motor] = run_steps[motor] * 1000 / (int32_t)eep_params.mode_sms_accel_count[motor];
                decel_steps_x1000[motor] = run_steps[motor] * 1000 / (int32_t)eep_params.mode_sms_decel_count[motor];
            }
            else
            {
                run_steps[motor] = 0;
                accel_steps_x1000[motor] = 0;
                decel_steps_x1000[motor] = 0;
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

bool bModeSmsIsFinished(void)
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

bool bModeSmsIsPaused(void)
{
    if (state == MODE_SMS_STATE_STOP) return false;
    
    return xTimerIsTimerActive(xModeSmsControlTimer) == pdFALSE;
}

bool bModeSmsIsRunning(void)
{
    return state != MODE_SMS_STATE_STOP;
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
        if ((state >= MODE_SMS_STATE_LOOP_BEGIN) && (eep_params.mode_sms_interval >= interval_timer))
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
    if (state >= MODE_SMS_STATE_LOOP_BEGIN && state <= MODE_SMS_STATE_LOOP_END)
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
                if (!eep_params.mode_sms_use_slider_as_shutter)
                {
                    state = MODE_SMS_STATE_WAIT_PRE_TIME;
                    SEGGER_RTT_printf(0, "ModeSms Control State Change: WAIT_PRE_TIME\n");
                    step_timer = 0;
                    remaining_step_time = eep_params.mode_sms_pre_time;
                }
                else
                {
                    state = MODE_SMS_STATE_OPEN_SHUTTER;
                    SEGGER_RTT_printf(0, "ModeSms Control State Change: OPEN_SHUTTER\n");
                }
                interval_timer = 0;
            }                
            break;
        case MODE_SMS_STATE_OPEN_SHUTTER:
            {
                if (test_mode == mode_smsTEST_NONE)
                {
                    // calculate available time for mode_sms movement
                    int32_t avail_move_time = eep_params.mode_sms_interval
                        - (eep_params.mode_sms_pre_time
                            + eep_params.mode_sms_post_time
                            + eep_params.mode_sms_focus_time
                            + eep_params.mode_sms_exposure_time);
                
                    // reduce used time on ~90% and divide by 4 (for acceleration and deceleration)
                    avail_move_time = avail_move_time * 90 / 400;

                    // sanity check
                    if (avail_move_time < 100) avail_move_time = 100;

                    int32_t steps = eep_params.mode_sms_positions[0].pos[0] - lSmGetPosition(0);
                    
                    if (eep_params.sm[0].power_save == 1) vSmEnable(0, 1);
                    if (eep_params.mode_sms_optimize_accel)
                    {
                        // calculate optimal acceleration
                        uint16_t accel = SM_STEPS_TO_MRAD(labs(steps)) * 1000000UL / (avail_move_time * avail_move_time);
                        
                        accel = utilsMIN(accel, SM_STEPS_TO_MRAD(eep_params.sm[0].accel_steps));
                        accel = utilsMAX(accel, SM_STEPS_TO_MRAD(SM_SPR/16));
                        uint16_t max_speed = accel * avail_move_time / 1000;
                        
                        ucSmMoveEx(0, steps, max_speed, accel, accel);
                    }
                    else
                    {
                        ucSmMove(0, steps);
                    }
                    state = MODE_SMS_STATE_WAIT_OPEN_SHUTTER;
                    SEGGER_RTT_printf(0, "ModeSms Control State Change: WAIT_OPEN_SHUTTER\n");
                }
                else
                {
                    state = MODE_SMS_STATE_WAIT_PRE_TIME;
                    SEGGER_RTT_printf(0, "ModeSms Control State Change: WAIT_PRE_TIME\n");
                    step_timer = 0;
                    remaining_step_time = eep_params.mode_sms_pre_time;
                }
            }
            break;
        case MODE_SMS_STATE_WAIT_OPEN_SHUTTER:
            {
                if (ucSmGetState(0) == SM_STATE_STOP)
                {
                    if (eep_params.sm[0].power_save == 1) vSmEnable(0, 0);
                    state = MODE_SMS_STATE_WAIT_PRE_TIME;
                    SEGGER_RTT_printf(0, "ModeSms Control State Change: WAIT_PRE_TIME\n");
                    step_timer = 0;
                    remaining_step_time = eep_params.mode_sms_pre_time;
                }
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
                    int32_t avail_move_time = eep_params.mode_sms_interval 
                        - (eep_params.mode_sms_pre_time 
                            + eep_params.mode_sms_post_time 
                            + eep_params.mode_sms_focus_time 
                            + eep_params.mode_sms_exposure_time);
                
                    if (!eep_params.mode_sms_use_slider_as_shutter)
                    {
                        // reduce used time on ~90% and divide by 2 (for acceleration and deceleration)
                        avail_move_time = avail_move_time * 90 / 200;
                    }
                    else
                    {
                        // reduce used time on ~90% and divide by 4 (for acceleration and deceleration)
                        avail_move_time = avail_move_time * 90 / 400;
                    }

                    // sanity check
                    if (avail_move_time < 100) avail_move_time = 100;

                    for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
                    {
                        int32_t steps = 0;

                        if (motor > 0 || !eep_params.mode_sms_use_slider_as_shutter)
                        {
                            if (current_step < eep_params.mode_sms_leadin_count[motor] + 1)
                            {
                                steps = 0;
                            }
                            else if (current_step < eep_params.mode_sms_leadin_count[motor] + eep_params.mode_sms_accel_count[motor])
                            {
                                steps = accel_steps_x1000[motor] * ((int32_t)current_step - (int32_t)eep_params.mode_sms_leadin_count[motor]) / 1000L;
                            }
                            else if (current_step < eep_params.mode_sms_count - eep_params.mode_sms_decel_count[motor] - eep_params.mode_sms_leadout_count[motor] + 1)
                            {
                                steps = run_steps[motor];
                            }
                            else if (current_step < eep_params.mode_sms_count - eep_params.mode_sms_leadout_count[motor])
                            {
                                steps = decel_steps_x1000[motor] * ((int32_t)eep_params.mode_sms_count - (int32_t)eep_params.mode_sms_leadout_count[motor] - (int32_t)current_step) / 1000L;
                            }
                            else
                            {
                                steps = 0;
                            }
                        
                            // prevent the motor to move beyond the end position
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
                        }
                        else
                        {
                            steps = eep_params.mode_sms_positions[1].pos[0] - lSmGetPosition(0);
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
                if (!eep_params.mode_sms_use_slider_as_shutter)
                {
                    state = MODE_SMS_STATE_WAIT_PRE_TIME;
                    SEGGER_RTT_printf(0, "ModeSms Control State Change: WAIT_PRE_TIME\n");
                    step_timer = 0;
                    remaining_step_time = eep_params.mode_sms_pre_time;
                }
                else
                {
                    state = MODE_SMS_STATE_OPEN_SHUTTER;
                    SEGGER_RTT_printf(0, "ModeSms Control State Change: OPEN_SHUTTER\n");
                }
                
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
