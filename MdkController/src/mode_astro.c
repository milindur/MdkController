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
#include "mode_astro.h"
#include "utils.h"

#define mode_astroCONTROL_TIMER_RATE     (10 / portTICK_RATE_MS)

static uint8_t state = MODE_ASTRO_STATE_STOP;
static uint8_t direction = MODE_ASTRO_DIR_NORTH;
static uint8_t speed = MODE_ASTRO_SPEED_SIDEREAL;
static bool finished = false;
static xTimerHandle xModeAstroControlTimer;

static void prvModeAstroControlCallback(void *pvParameters);

void vModeAstroInit(void)
{
    xModeAstroControlTimer = xTimerCreate(
        (const char * const) "ModeAstroCtrlTmr",
        mode_astroCONTROL_TIMER_RATE,
        pdTRUE,
        NULL,
        prvModeAstroControlCallback);
    configASSERT(xModeAstroControlTimer);
}

void vModeAstroPause(void)
{
    if (state == MODE_ASTRO_STATE_STOP) return;
    if (bModeAstroIsPaused()) return;
    
    xTimerStop(xModeAstroControlTimer, 0);
}

void vModeAstroResume(void)
{
    if (state == MODE_ASTRO_STATE_STOP) return;
    if (!bModeAstroIsPaused()) return;
    
    xTimerStart(xModeAstroControlTimer, 0);
}

void vModeAstroStart(uint8_t dir, uint8_t spd)
{
    if (state != MODE_ASTRO_STATE_STOP) return;
    
    taskENTER_CRITICAL();
    {
        state = MODE_ASTRO_STATE_WAKE_SM;
        direction = dir;
        speed = spd;
        xTimerStart(xModeAstroControlTimer, 0);
    }
    taskEXIT_CRITICAL();
}

void vModeAstroStop(void)
{
    taskENTER_CRITICAL();
    {
        xTimerStop(xModeAstroControlTimer, 0);
        
        for (uint8_t motor = 0; motor < 1; motor++)
        {
            vSmStop(motor);
            if (eep_params.sm[motor].power_save == 0) vSmEnable(motor, 0);
        }
        vCamClear();
        state = MODE_ASTRO_STATE_STOP;
        finished = true;
    }
    taskEXIT_CRITICAL();
}


bool bModeAstroIsFinished(void)
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

bool bModeAstroIsPaused(void)
{
    if (state == MODE_ASTRO_STATE_STOP) return false;
    
    return xTimerIsTimerActive(xModeAstroControlTimer) == pdFALSE;
}

bool bModeAstroIsRunning(void)
{
    return state != MODE_ASTRO_STATE_STOP;
}

uint8_t ucModeAstroGetState(void)
{
    uint8_t v;
    
    taskENTER_CRITICAL();
    {
        v = state;
    }
    taskEXIT_CRITICAL();
    
    return v;
}

static void prvModeAstroControlCallback(void *pvParameters)
{
    switch (state)
    {
        case MODE_ASTRO_STATE_STOP:
            xTimerStop(xModeAstroControlTimer, 0);
            break;
        case MODE_ASTRO_STATE_WAKE_SM:
            {
                for (uint8_t motor = 0; motor < 1; motor++)
                {
                    vSmEnable(motor, 2);
                }
            }
            state = MODE_ASTRO_STATE_BACKLASH_1;
            SEGGER_RTT_printf(0, "ModeAstro Control State Change: BACKLASH_1\n");
            break;
        case MODE_ASTRO_STATE_BACKLASH_1:
            {
                int32_t steps = direction == MODE_ASTRO_DIR_NORTH ? -1 * SM_SPR : SM_SPR;

                for (uint8_t motor = 0; motor < 1; motor++)
                {
                    ucSmMove(motor, steps);
                }
            }
            state = MODE_ASTRO_STATE_WAIT_BACKLASH_1;
            SEGGER_RTT_printf(0, "ModeAstro Control State Change: WAIT_BACKLASH_1\n");
            break;
        case MODE_ASTRO_STATE_WAIT_BACKLASH_1:
            {
                bool all_stoped = true;
                for (uint8_t motor = 0; motor < 1; motor++)
                {
                    if (ucSmGetState(motor) != SM_STATE_STOP)
                    {
                        all_stoped = false;
                        continue;
                    }
                }
                if (all_stoped)
                {
                    state = MODE_ASTRO_STATE_BACKLASH_2;
                    SEGGER_RTT_printf(0, "ModeAstro Control State Change: BACKLASH_2\n");
                }
            }
            break;
        case MODE_ASTRO_STATE_BACKLASH_2:
            {
                int32_t steps = direction == MODE_ASTRO_DIR_NORTH ? SM_SPR : -1 * SM_SPR;

                for (uint8_t motor = 0; motor < 1; motor++)
                {
                    ucSmMove(motor, steps);
                }
            }
            state = MODE_ASTRO_STATE_WAIT_BACKLASH_2;
            SEGGER_RTT_printf(0, "ModeAstro Control State Change: WAIT_BACKLASH_2\n");
            break;
        case MODE_ASTRO_STATE_WAIT_BACKLASH_2:
            {
                bool all_stoped = true;
                for (uint8_t motor = 0; motor < 1; motor++)
                {
                    if (ucSmGetState(motor) != SM_STATE_STOP)
                    {
                        all_stoped = false;
                        continue;
                    }
                }
                if (all_stoped)
                {
                    state = MODE_ASTRO_STATE_MOVE;
                    SEGGER_RTT_printf(0, "ModeAstro Control State Change: MOVE\n");
                }
            }
            break;
        case MODE_ASTRO_STATE_MOVE:
            {
                bSmMoveContinuousAstro(0, 
                    direction == MODE_ASTRO_DIR_NORTH ? SM_CW : SM_CCW, 
                    speed == MODE_ASTRO_SPEED_SIDEREAL ? SM_ASTRO_SIDEREAL : SM_ASTRO_LUNAR);
                
                state = MODE_ASTRO_STATE_WAIT_MOVE;
                SEGGER_RTT_printf(0, "ModeAstro Control State Change: WAIT_MOVE\n");
            }
            break;
        case MODE_ASTRO_STATE_WAIT_MOVE:
            {
                bool all_stoped = true;
                for (uint8_t motor = 0; motor < 1; motor++)
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
                    state = MODE_ASTRO_STATE_SLEEP_SM;
                    SEGGER_RTT_printf(0, "ModeAstro Control State Change: WAIT_PRE_TIME\n");
                }
            }
            break;
        case MODE_ASTRO_STATE_SLEEP_SM:
            {
                for (uint8_t motor = 0; motor < 1; motor++)
                {
                    if (eep_params.sm[motor].power_save == 1) vSmEnable(motor, 0);
                }
            }
            state = MODE_ASTRO_STATE_STOP;
            finished = true;
            SEGGER_RTT_printf(0, "ModeAstro Control State Change: STOP\n");
            break;
    }
}
