#include "FreeRTOS.h"
#include "task.h"

#include "asf.h"

#include "trcUser.h"
#include "trcConfig.h"
#include "trcHardwarePort.h"

#include "segger_rtt.h"
#include "eep.h"
#include "sm.h"
#include "utils.h"

#define SM_PIN_ENABLE_1   PIO_PB25_IDX
#define SM_PIN_STEP_1     PIO_PC28_IDX
#define SM_PIN_DIR_1      PIO_PC26_IDX
#define SM_PIN_ENABLE_2   PIO_PC25_IDX
#define SM_PIN_STEP_2     PIO_PC24_IDX
#define SM_PIN_DIR_2      PIO_PC23_IDX
#define SM_PIN_ENABLE_3   PIO_PD7_IDX
#define SM_PIN_STEP_3     PIO_PD8_IDX
#define SM_PIN_DIR_3      PIO_PB27_IDX
#define SM_PIN_ENABLE_4   PIO_PA16_IDX
#define SM_PIN_STEP_4     PIO_PA24_IDX
#define SM_PIN_DIR_4      PIO_PA23_IDX

#define SM_PIN_ENABLE(motor)   SM_PIN_ENABLE_##motor
#define SM_PIN_STEP(motor)     SM_PIN_STEP_##motor
#define SM_PIN_DIR(motor)      SM_PIN_DIR_##motor

#define SM_PIN_DBG			PIO_PC29_IDX

#define T_PRESCALE  32

#define T_FREQ      (84000000/T_PRESCALE)
#define ALPHA       (2*M_PI/SM_SPR)                     // 2*PI / SPR
#define A_T_x1000   ((int32_t)(ALPHA*T_FREQ*1000))      // (ALPHA * T_FREQ)*1000
#define T_FREQ_148  ((int16_t)((T_FREQ*0.676)/100))     // divided by 100 and scaled by 0.676
#define A_SQ        ((int64_t)(ALPHA*2*100000000000))   // ALPHA*2*100000000000
#define A_x200000   ((int64_t)(ALPHA*2*100000))         // ALPHA*2*100000

#define CW  1
#define CCW 0

typedef struct
{
	volatile int32_t position;
	volatile int32_t remaining_steps;
	uint16_t accel_mrad;
	uint16_t decel_mrad;
	uint16_t speed_max_mrad;
	
	uint8_t pin_enable;
	uint8_t pin_step;
	uint8_t pin_dir;
	
	volatile uint8_t run_state:3;
	volatile uint8_t dir:1;
	volatile uint32_t step_delay;
	volatile uint32_t min_step_delay;
	volatile int32_t accel_val;
	volatile int32_t decel_val;
	volatile uint32_t decel_start;
	volatile int32_t accel_decel_step_count;
	
	// Remember the last step delay used when accelerating.
	uint32_t last_accel_delay;
	// Counting steps when moving.
	uint32_t step_count;
	// Keep track of remainder from new_step_delay calculation to increase accuracy
	int32_t rest;
	
	volatile float_t speed_cont_current_mrad;
	volatile float_t speed_cont_target_mrad;
	float_t cont_esum;
	float_t cont_ealt;
	bool stop_cont;
} sm_state_t;

static sm_state_t _state[SM_MOTORS_USED];

static uint32_t prulFastSqrt(uint32_t v);
static inline void prvStep(uint8_t motor, uint8_t dir);
static inline void prvEnableFromISR(uint8_t motor, uint8_t enable);
static void prvContinuousControlTask(void *pvParameters);

void vSmInit(void)
{
	sysclk_enable_peripheral_clock(ID_TC0);
	sysclk_enable_peripheral_clock(ID_TC1);
	sysclk_enable_peripheral_clock(ID_TC2);

	_state[0].pin_enable = SM_PIN_ENABLE(1);
	_state[0].pin_step = SM_PIN_STEP(1);
	_state[0].pin_dir = SM_PIN_DIR(1);
	_state[1].pin_enable = SM_PIN_ENABLE(3);
	_state[1].pin_step = SM_PIN_STEP(3);
	_state[1].pin_dir = SM_PIN_DIR(3);
	_state[2].pin_enable = SM_PIN_ENABLE(4);
	_state[2].pin_step = SM_PIN_STEP(4);
	_state[2].pin_dir = SM_PIN_DIR(4);

	for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
	{
		_state[motor].position = 0;
		_state[motor].run_state = SM_STATE_STOP;

		tc_init(TC0, motor, TC_CMR_TCCLKS_TIMER_CLOCK3 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);
		tc_write_rc(TC0, motor, 0);
		tc_enable_interrupt(TC0, motor, TC_IER_CPCS);

		ioport_set_pin_dir(_state[motor].pin_enable, IOPORT_DIR_OUTPUT);
		ioport_set_pin_mode(_state[motor].pin_enable, 0);
		ioport_set_pin_dir(_state[motor].pin_step, IOPORT_DIR_OUTPUT);
		ioport_set_pin_dir(_state[motor].pin_dir, IOPORT_DIR_OUTPUT);
	
		ioport_set_pin_level(_state[motor].pin_enable, false);
		ioport_set_pin_level(_state[motor].pin_step, false);
		ioport_set_pin_level(_state[motor].pin_dir, false);

		vSmEnable(motor, 1);
		delay_ms(2);
		vSmEnable(motor, 0);
	}
	
	NVIC_EnableIRQ(TC0_IRQn);
	NVIC_EnableIRQ(TC1_IRQn);
	NVIC_EnableIRQ(TC2_IRQn);
	NVIC_SetPriority(TC0_IRQn, 0);
	NVIC_SetPriority(TC1_IRQn, 0);
	NVIC_SetPriority(TC2_IRQn, 0);
	
	vTraceSetISRProperties(1, "TC0_Handler", 0);
	vTraceSetISRProperties(2, "TC1_Handler", 0);
	vTraceSetISRProperties(3, "TC2_Handler", 0);
	
	ioport_set_pin_dir(SM_PIN_DBG, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(SM_PIN_DBG, false);
	
	vSmReload();
	
	xTaskCreate(prvContinuousControlTask, "SmContCtrl", 200, NULL, 1, NULL);
}

void vSmReload(void)
{
	for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
	{
		eep_params.sm[motor].accel_steps = utilsMIN(eep_params.sm[motor].accel_steps, SM_MAX_ACCEL_STEPS);
		eep_params.sm[motor].decel_steps = utilsMIN(eep_params.sm[motor].decel_steps, SM_MAX_ACCEL_STEPS);
		eep_params.sm[motor].speed_max_steps = utilsMIN(eep_params.sm[motor].speed_max_steps, SM_MAX_SPEED_STEPS);

		_state[motor].accel_mrad = SM_STEPS_TO_MRAD(eep_params.sm[motor].accel_steps);
		_state[motor].decel_mrad = SM_STEPS_TO_MRAD(eep_params.sm[motor].decel_steps);
		_state[motor].speed_max_mrad = SM_STEPS_TO_MRAD(eep_params.sm[motor].speed_max_steps);
	}
}

void vSmEnable(uint8_t motor, uint8_t enable)
{
	sm_state_t * state = &_state[motor];

	taskENTER_CRITICAL();
	
	if (enable)
	{
		ioport_set_pin_dir(state->pin_enable, IOPORT_DIR_OUTPUT);
		ioport_set_pin_mode(state->pin_enable, 0);
		ioport_set_pin_level(state->pin_enable, false);
	}
	else
	{
		ioport_set_pin_dir(state->pin_enable, IOPORT_DIR_INPUT);
		ioport_set_pin_mode(state->pin_enable, 0);
		ioport_set_pin_level(state->pin_enable, true);
	}

	taskEXIT_CRITICAL();
}

void prvEnableFromISR(uint8_t motor, uint8_t enable)
{
	sm_state_t * state = &_state[motor];

	//taskDISABLE_INTERRUPTS();

	if (enable)
	{
		ioport_set_pin_dir(state->pin_enable, IOPORT_DIR_OUTPUT);
		ioport_set_pin_mode(state->pin_enable, 0);
		ioport_set_pin_level(state->pin_enable, false);
	}
	else
	{
		ioport_set_pin_dir(state->pin_enable, IOPORT_DIR_INPUT);
		ioport_set_pin_mode(state->pin_enable, 0);
		ioport_set_pin_level(state->pin_enable, true);
	}

	//taskENABLE_INTERRUPTS();
}

void vSmSetParams(uint16_t accel, uint16_t decel, uint16_t speed_max,
		uint16_t reverse, uint16_t power_save)
{
	eep_params.sm[0].accel_steps = accel;
	eep_params.sm[0].decel_steps = decel;
	eep_params.sm[0].speed_max_steps = speed_max;
	eep_params.sm[0].motor_reverse = reverse;
	eep_params.sm[0].power_save = (uint8_t) power_save;
	vEepSave();

	vSmReload();
}

void vSmGetParams(uint16_t *accel, uint16_t *decel, uint16_t *speed_max,
		uint16_t *reverse, uint16_t *power_save)
{
	*accel = eep_params.sm[0].accel_steps;
	*decel = eep_params.sm[0].decel_steps;
	*speed_max = eep_params.sm[0].speed_max_steps;
	*reverse = eep_params.sm[0].motor_reverse;
	*power_save = eep_params.sm[0].power_save;
}

bool bSmMoveContinuous(uint8_t motor, int32_t speed)
{
	taskENTER_CRITICAL();
	
	uint8_t sm_state = ucSmGetState(motor);
	
	if ((sm_state != SM_STATE_STOP) && (sm_state != SM_STATE_CONT))
	{
		return 1;
	}
	
	sm_state_t * state = &_state[motor];

	int32_t speed_mrad;
	if (speed >= 0)
	{
		speed_mrad = SM_STEPS_TO_MRAD(speed);
		if (speed_mrad > state->speed_max_mrad) speed_mrad = state->speed_max_mrad;
	}
	else
	{
		speed_mrad = -1 * SM_STEPS_TO_MRAD(-1 * speed);
		if (speed_mrad < -1 * (int32_t) state->speed_max_mrad) speed_mrad = -1 * (int32_t) state->speed_max_mrad;
	}
		 
	SEGGER_RTT_printf(0, "move cont: %d\n", speed_mrad);

	if (sm_state == SM_STATE_STOP)
	{
		state->run_state = SM_STATE_CONT;
		state->speed_cont_current_mrad = 0;
		state->speed_cont_target_mrad = speed_mrad;
		state->step_delay = A_T_x1000 / SM_STEPS_TO_MRAD(0.5*SM_SPR);
		
		if (eep_params.sm[motor].power_save == 1)
		{
			vSmEnable(motor, 1);
		}

		// Run Timer/Counter.
		tc_write_rc(TC0, motor, 10);
		tc_start(TC0, motor);
	}
	else
	{
		state->speed_cont_target_mrad = speed_mrad;
		if (speed == 0)
		{
			state->stop_cont = true;
		}
	}
	
	taskEXIT_CRITICAL();
	
	return true;
}

void prvContinuousControlTask(void *pvParameters)
{
	#define Ta 0.02f
	#define Kp 0.06f
	#define Ki 0.04f
	#define Kd 0.00f
	
	float_t current_accel = 0.0f;
	
	portTickType xLastWakeTime;
	
	UNUSED(pvParameters);
	
	xLastWakeTime = xTaskGetTickCount();
	
	for (;;)
	{
		for (uint8_t motor = 0; motor < SM_MOTORS_USED; motor++)
		{
			sm_state_t * state = &_state[motor];
			
			if (state->run_state != SM_STATE_CONT) continue;
			
			float_t w = state->speed_cont_target_mrad;
			float_t x = state->speed_cont_current_mrad;
			
			/*if (((int32_t) w < 0 && (int32_t) x > 0) || ((int32_t) w > 0 && (int32_t) x < 0))
			{
				w = 0;
			}*/
			
			float_t e = w - x;
			state->cont_esum = state->cont_esum + e;
			
			if (state->cont_esum > (float_t) state->accel_mrad)
			{
				state->cont_esum = (float_t) state->accel_mrad;
			}
			else if (state->cont_esum < -1.0f * (float_t) state->accel_mrad)
			{
				state->cont_esum = -1.0f * (float_t) state->accel_mrad;
			}
			
			float_t y = Kp*e + Ki*Ta*state->cont_esum + Kd/Ta*(e - state->cont_ealt);
			
			/*current_accel = current_accel + y;
			if (current_accel > (float_t) state->accel_mrad) current_accel = (float_t) state->accel_mrad;
			if (current_accel < -1.0f * (float_t) state->accel_mrad) current_accel = -1.0f * (float_t) state->accel_mrad;*/
			
			current_accel = y / Ta;
			
			if (current_accel > (float_t) state->accel_mrad)
			{
				y = (float_t) state->accel_mrad * Ta;
			}
			if (current_accel < (-1.0f * (float_t) state->decel_mrad))
			{
				y = -1.0f * (float_t) state->accel_mrad * Ta;
			}			
			
			taskENTER_CRITICAL();
			state->speed_cont_current_mrad = state->speed_cont_current_mrad + y;

			/*state->speed_cont_current_mrad = state->speed_cont_current_mrad + current_accel * Ta;*/
			if (state->speed_cont_current_mrad > state->speed_max_mrad) state->speed_cont_current_mrad = state->speed_max_mrad;
			if (state->speed_cont_current_mrad < -1.0 * (float_t) state->speed_max_mrad) state->speed_cont_current_mrad = -1.0 * (float_t) state->speed_max_mrad;
			
			if (fabs(state->speed_cont_current_mrad) > 150.0f)
			{
				state->step_delay = A_T_x1000 / labs((int32_t) state->speed_cont_current_mrad);
			}
			else
			{
				//state->speed_cont_current_mrad = 0;
				state->step_delay = A_T_x1000 / 75;
				if (fabs(w) <= 1.0f)
				{
					state->speed_cont_current_mrad = 0.0f;
					state->cont_esum = 0.0f;
					state->cont_ealt = 0.0f;
				}
			}
			taskEXIT_CRITICAL();

			if (/*(int32_t) e != 0 &&*/ motor == 0)
			{
				SEGGER_RTT_printf(0, "move cont control: w=%d x=%d e=%d esum=%d ealt=%d y=%d curacc=%d curspd=%d\n", (int32_t) w, (int32_t) x, (int32_t) e, (int32_t) state->cont_esum, (int32_t) state->cont_ealt, (int32_t) y, (int32_t) current_accel, (int32_t) state->speed_cont_current_mrad);
			}

			state->cont_ealt = e;
		}
		
		vTaskDelayUntil(&xLastWakeTime, 20 / portTICK_RATE_MS);
	}
}

uint8_t ucSmStart(uint8_t motor, uint8_t dir)
{
	if (dir == CW)
	{
		return ucSmMove(motor, INT32_MAX);
	}
	else
	{
		return ucSmMove(motor, INT32_MIN + 1);
	}
}

uint8_t ucSmMove(uint8_t motor, int32_t step)
{
	return ucSmMoveEx(motor, step, _state[motor].speed_max_mrad, _state[motor].accel_mrad, _state[motor].decel_mrad);
}

uint8_t ucSmMoveEx(uint8_t motor, int32_t step, uint16_t speed, uint16_t accel, uint16_t decel)
{
	if (ucSmGetState(motor) != SM_STATE_STOP)
	{
		return 1;
	}
	
	sm_state_t * state = &_state[motor];

	SEGGER_RTT_printf(0, "move: %d %u %u %u\n", step, speed, accel, decel);

	int32_t max_s_lim;
	int32_t accel_lim;
	
	state->remaining_steps = step;

	if (step < 0)
	{
		state->dir = CCW;
		step = -step;
		if (!eep_params.sm[motor].motor_reverse)
		{
			ioport_set_pin_level(state->pin_dir, false);
		}
		else
		{
			ioport_set_pin_level(state->pin_dir, true);
		}
	}
	else
	{
		state->dir = CW;
		if (!eep_params.sm[motor].motor_reverse)
		{
			ioport_set_pin_level(state->pin_dir, true);
		}
		else
		{
			ioport_set_pin_level(state->pin_dir, false);
		}
	}

	// If moving only 1 step.
	if (step == 1)
	{
		// Move one step...
		state->accel_decel_step_count = -1;
		// ...in SM_STATE_DECEL state.
		state->run_state = SM_STATE_DECEL;
		// Just a short delay so main() can act on 'running'.
		state->step_delay = 1000;

		if (eep_params.sm[motor].power_save == 1)
		{
			vSmEnable(motor, 1);
		}

		// Run Timer/Counter.
		tc_write_rc(TC0, motor, 10);
		tc_start(TC0, motor);
	}
	// Only move if number of steps to move is not zero.
	else if (step != 0)
	{
		// Set max speed limit, by calc min_delay to use in timer.
		// min_delay = (alpha / tt) / w
		state->min_step_delay = A_T_x1000 / speed;

		// Set acceleration by calc the first step delay.
		uint64_t step_delay_tmp = (T_FREQ_148 * prulFastSqrt(A_SQ / accel)) / 100;
		if (step_delay_tmp > INT32_MAX)
		{
			step_delay_tmp = INT32_MAX;
		}
		state->step_delay = step_delay_tmp;

		// Find out after how many steps does the speed hit the max speed limit.
		max_s_lim = (int64_t) speed * (int64_t) speed / ((A_x200000 * (int64_t) accel) / 100);
		// If we hit max speed limit before 0,5 step it will round to 0.
		// But in practice we need to move at least 1 step to get any speed at all.
		if (max_s_lim == 0)
		{
			max_s_lim = 1;
		}

		// Find out after how many steps we must start deceleration.
		accel_lim = ((int64_t) step * (int64_t) decel) / ((int64_t) accel + (int64_t) decel);
		// We must accelerate at least 1 step before we can start deceleration.
		if (accel_lim == 0)
		{
			accel_lim = 1;
		}

		// Use the limit we hit first to calc decel.
		if (accel_lim <= max_s_lim)
		{
			state->accel_val = accel_lim;
			state->decel_val = (int32_t) accel_lim - step;
		}
		else
		{
			state->accel_val = max_s_lim;
			state->decel_val = -((int64_t) max_s_lim * (int64_t) accel) / (int64_t) decel;
		}
		// We must decelerate at least 1 step to stop.
		if (state->decel_val == 0)
		{
			state->decel_val = -1;
		}

		// Find step to start deceleration.
		state->decel_start = step + state->decel_val;

		// If the maximum speed is so low that we don't need to go via acceleration state.
		if (state->step_delay <= state->min_step_delay)
		{
			state->step_delay = state->min_step_delay;
			state->run_state = SM_STATE_RUN;
		}
		else
		{
			state->run_state = SM_STATE_ACCEL;
		}

		// Reset counter.
		state->accel_decel_step_count = 0;

		SEGGER_RTT_printf(0, "move start: %u %u %d %d %d %u\n", state->min_step_delay, state->step_delay, state->accel_decel_step_count, state->accel_val, state->decel_val, state->decel_start);

		if (eep_params.sm[motor].power_save == 1)
		{
			vSmEnable(motor, 1);
		}

		// Set Timer/Counter.
		tc_write_rc(TC0, motor, 10);
		tc_start(TC0, motor);
	}

	return 0;
}

void vSmStop(uint8_t motor)
{
	sm_state_t * state = &_state[motor];
	
	taskENTER_CRITICAL();

	switch (state->run_state)
	{
	case SM_STATE_STOP:
	case SM_STATE_DECEL:
		break;
	case SM_STATE_ACCEL:
		state->accel_decel_step_count = -1 * (int64_t) state->accel_decel_step_count * (int64_t) state->accel_mrad / (int64_t) state->decel_mrad;
		state->run_state = SM_STATE_DECEL;
		break;
	case SM_STATE_RUN:
		state->accel_decel_step_count = state->decel_val;
		state->run_state = SM_STATE_DECEL;
		break;
	}

	taskEXIT_CRITICAL();
}

void vSmEmergencyStop(uint8_t motor)
{
	taskENTER_CRITICAL();
	_state[motor].run_state = SM_STATE_STOP;
	taskEXIT_CRITICAL();
}

int32_t lSmGetRemainingSteps(uint8_t motor)
{
	int32_t v;
	
	taskENTER_CRITICAL();
	v = _state[motor].remaining_steps;
	taskEXIT_CRITICAL();

	return v;
}

int32_t lSmGetPosition(uint8_t motor)
{
	int32_t v;

	taskENTER_CRITICAL();
	v = _state[motor].position;
	taskEXIT_CRITICAL();

	return v;
}

void vSmResetPosition(uint8_t motor)
{
	taskENTER_CRITICAL();
	_state[motor].position = 0;
	taskEXIT_CRITICAL();
}

uint8_t ucSmGetState(uint8_t motor)
{
	uint8_t v;

	taskENTER_CRITICAL();
	v = _state[motor].run_state;
	taskEXIT_CRITICAL();

	return v;
}

static inline void prvStep(uint8_t motor, uint8_t dir)
{
	sm_state_t * state = &_state[motor];
	
	//ioport_toggle_pin_level(SM_PIN_DBG);
	//ioport_set_pin_level(SM_PIN_DBG, false);
	
	if (dir == CW)
	{
		if (!eep_params.sm[motor].motor_reverse)
		{
			ioport_set_pin_level(state->pin_dir, true);
		}
		else
		{
			ioport_set_pin_level(state->pin_dir, false);
		}
		state->position++;
		state->remaining_steps--;
	}
	else
	{
		if (!eep_params.sm[motor].motor_reverse)
		{
			ioport_set_pin_level(state->pin_dir, false);
		}
		else
		{
			ioport_set_pin_level(state->pin_dir, true);
		}
		state->position--;
		state->remaining_steps++;
	}

	//ioport_toggle_pin_level(SM_PIN_DBG);

	//delay_us(1);
	pio_set_pin_high(state->pin_step);
	//ioport_set_pin_level(state->pin_step, true);
	//delay_us(2);
	pio_set_pin_low(state->pin_step);
	//ioport_set_pin_level(state->pin_step, false);
	//delay_us(1);
	
	//ioport_toggle_pin_level(SM_PIN_DBG);
	//ioport_set_pin_level(SM_PIN_DBG, true);
}

/* Square root routine.
 *
 * sqrt routine 'grupe', from comp.sys.ibm.pc.programmer
 * Subject: Summary: SQRT(int) algorithm (with profiling)
 *    From: warwick@cs.uq.oz.au (Warwick Allison)
 *    Date: Tue Oct 8 09:16:35 1991
 */
static uint32_t prulFastSqrt(uint32_t x)
{
	register uint32_t xr;  // result register
	register uint32_t q2;  // scan-bit register
	register uint8_t f;    // flag (one bit)

	xr = 0;                     // clear result
	q2 = 0x40000000L;           // highest possible result bit
	do
	{
		if ((xr + q2) <= x)
		{
			x -= xr + q2;
			f = 1;              // set flag
		}
		else
		{
			f = 0;              // clear flag
		}
		xr >>= 1;
		if (f)
		{
			xr += q2;           // test flag
		}
	} while (q2 >>= 2);         // shift twice

	if (xr < x)
	{
		return xr + 1;          // add for rounding
	}
	else
	{
		return xr;
	}
}

void prvSmIsrHandler(uint8_t motor)
{
	// Holds next delay period.
	uint32_t new_step_delay;
	sm_state_t * state = &_state[motor];

	tc_write_rc(TC0, motor, state->step_delay);

	switch (state->run_state)
	{
	default:
	case SM_STATE_STOP:
		state->remaining_steps = 0;
		state->step_count = 0;
		new_step_delay = 0;
		state->rest = 0;
		if (eep_params.sm[motor].power_save == 1)
		{
			prvEnableFromISR(motor, 0);
		}
		tc_stop(TC0, motor);
		SEGGER_RTT_printf(0, "move timer: stop\r\n");
		break;
	case SM_STATE_ACCEL:
		prvStep(motor, state->dir);
		state->step_count++;
		state->accel_decel_step_count++;
		new_step_delay = (int32_t) state->step_delay - (2 * (int32_t) state->step_delay + state->rest) / (4 * state->accel_decel_step_count + 1);
		if (new_step_delay > INT32_MAX)
		{
			new_step_delay = INT32_MAX;
		}
		state->rest = ((2 * (int32_t) state->step_delay) + state->rest) % (4 * state->accel_decel_step_count + 1);
		// Check if we should start deceleration.
		if (state->step_count >= state->decel_start)
		{
			state->accel_decel_step_count = state->decel_val;
			state->run_state = SM_STATE_DECEL;
		}
		// Check if we hit max speed.
		else if (new_step_delay <= state->min_step_delay)
		{
			state->last_accel_delay = new_step_delay;
			new_step_delay = state->min_step_delay;
			state->rest = 0;
			state->run_state = SM_STATE_RUN;
		}
		break;
	case SM_STATE_RUN:
		prvStep(motor, state->dir);
		state->step_count++;
		new_step_delay = state->min_step_delay;
		// Check if we should start deceleration.
		if (state->step_count >= state->decel_start)
		{
			state->accel_decel_step_count = state->decel_val;
			// Start deceleration with same delay as accel ended with.
			new_step_delay = state->last_accel_delay;
			state->run_state = SM_STATE_DECEL;
		}
		break;
	case SM_STATE_DECEL:
		prvStep(motor, state->dir);
		state->step_count++;
		state->accel_decel_step_count++;
		new_step_delay = (int32_t) state->step_delay - (2 * (int32_t) state->step_delay + state->rest) / (4 * state->accel_decel_step_count + 1);
		if (new_step_delay > INT32_MAX)
		{
			new_step_delay = INT32_MAX;
		}
		state->rest = (2 * (int32_t) state->step_delay + state->rest) % (4 * state->accel_decel_step_count + 1);
		// Check if we at last step
		if (state->accel_decel_step_count >= 0)
		{
			new_step_delay = UINT32_MAX;
			state->run_state = SM_STATE_STOP;
		}
		break;
	case SM_STATE_CONT:
		{
			int32_t cur = (int32_t) state->speed_cont_current_mrad;
			if (cur != 0)
			{
				if (cur > 0) state->dir = SM_CW;
				if (cur < 0) state->dir = SM_CCW;
				prvStep(motor, state->dir);
				//SEGGER_RTT_printf(0, "%d", motor);
			}
			else
			{
				if (state->stop_cont)
				{
					new_step_delay = UINT32_MAX;
					state->run_state = SM_STATE_STOP;
					state->stop_cont = false;
				}
			}
			new_step_delay = state->step_delay;
		}
		break;
	}

	state->step_delay = new_step_delay;
	
	/*if (state->step_delay == 0)
	{
		SEGGER_RTT_printf(0, "move timer: %u\r\n", state->step_delay);
	}*/

	//SEGGER_RTT_printf(0, "move timer: %u %u %u %d %d %d %u\r\n", step_count, state->min_step_delay, state->step_delay, state->accel_decel_step_count, state->accel_val, state->decel_val, state->decel_start);
	//SEGGER_RTT_printf(0, "move timer: %u\r\n", state->step_delay);
	//SEGGER_RTT_printf(0, "%d", motor);
}

void TC0_Handler(void)
{
	vTraceStoreISRBegin(1);
	
	if ((tc_get_status(TC0, 0) & TC_SR_CPCS) == TC_SR_CPCS)
	{
		prvSmIsrHandler(0);
	}
	
	vTraceStoreISREnd(0);
}

void TC1_Handler(void)
{
	vTraceStoreISRBegin(2);

	if ((tc_get_status(TC0, 1) & TC_SR_CPCS) == TC_SR_CPCS)
	{
		prvSmIsrHandler(1);
	}
	
	vTraceStoreISREnd(0);
}

void TC2_Handler(void)
{
	vTraceStoreISRBegin(3);

	if ((tc_get_status(TC0, 2) & TC_SR_CPCS) == TC_SR_CPCS)
	{
		prvSmIsrHandler(2);
	}
	
	vTraceStoreISREnd(0);
}
