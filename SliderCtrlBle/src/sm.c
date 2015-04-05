#include "FreeRTOS.h"
#include "task.h"
#include "asf.h"
#include "segger_rtt.h"
#include "eep.h"
#include "sm.h"

#define SM_MOTOR_SHIELD 1

#if (SM_MOTOR_SHIELD == 1)
	#define SM_PIN_ENABLE   PIO_PB25_IDX
	#define SM_PIN_STEP     PIO_PC28_IDX
	#define SM_PIN_DIR      PIO_PC26_IDX
#endif
#if (SM_MOTOR_SHIELD == 2)
	#define SM_PIN_ENABLE   PIO_PC25_IDX
	#define SM_PIN_STEP     PIO_PC24_IDX
	#define SM_PIN_DIR      PIO_PC23_IDX
#endif
#if (SM_MOTOR_SHIELD == 3)
	#define SM_PIN_ENABLE   PIO_PD7_IDX
	#define SM_PIN_STEP     PIO_PD8_IDX
	#define SM_PIN_DIR      PIO_PB27_IDX
#endif
#if (SM_MOTOR_SHIELD == 4)
	#define SM_PIN_ENABLE   PIO_PA16_IDX
	#define SM_PIN_STEP     PIO_PA24_IDX
	#define SM_PIN_DIR      PIO_PA23_IDX
#endif

#define T_PRESCALE  32

#define T_FREQ      (84000000/T_PRESCALE)
#define ALPHA       (2*M_PI/SM_SPR)                     // 2*PI / SPR
#define A_T_x1000   ((int32_t)(ALPHA*T_FREQ*1000))      // (ALPHA * T_FREQ)*1000
#define T_FREQ_148  ((int16_t)((T_FREQ*0.676)/100))     // divided by 100 and scaled by 0.676
#define A_SQ        ((int64_t)(ALPHA*2*100000000000))   // ALPHA*2*100000000000
#define A_x200000   ((int64_t)(ALPHA*2*100000))         // ALPHA*2*100000

#define CW  1
#define CCW 0

static sm_state_t *_state;

static uint32_t l_sqrt(uint32_t v);
static void sm_step(uint8_t dir);
void sm_enable_from_isr(uint8_t enable);

void sm_init(sm_state_t *state)
{
	_state = state;
	
	_state->position = 0;
	_state->run_state = SM_STATE_STOP;
	
	_state->pin_enable = SM_PIN_ENABLE;
	_state->pin_step = SM_PIN_STEP;
	_state->pin_dir = SM_PIN_DIR;

	sysclk_enable_peripheral_clock(ID_TC0);

	tc_init(TC0, 0, TC_CMR_TCCLKS_TIMER_CLOCK3 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);
	tc_write_rc(TC0, 0, 1312500);
	
	NVIC_EnableIRQ(TC0_IRQn);
	
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
	
	ioport_set_pin_dir(_state->pin_enable, IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(_state->pin_enable, 0);
	ioport_set_pin_dir(_state->pin_step, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(_state->pin_dir, IOPORT_DIR_OUTPUT);

	ioport_set_pin_level(_state->pin_enable, false);
	ioport_set_pin_level(_state->pin_step, false);
	ioport_set_pin_level(_state->pin_dir, false);
	
	sm_enable(1);
	delay_ms(2);
	sm_enable(0);

	sm_reload();
}

void sm_reload(void)
{
	if (eep_params.sm_accel_steps > SM_MAX_ACCEL_STEPS)
	{
		eep_params.sm_accel_steps = SM_MAX_ACCEL_STEPS;
	}
	if (eep_params.sm_decel_steps > SM_MAX_ACCEL_STEPS)
	{
		eep_params.sm_decel_steps = SM_MAX_ACCEL_STEPS;
	}
	if (eep_params.sm_speed_max_steps > SM_MAX_SPEED_STEPS)
	{
		eep_params.sm_speed_max_steps = SM_MAX_SPEED_STEPS;
	}

	_state->accel_mrad = SM_STEPS_TO_MRAD(eep_params.sm_accel_steps);
	_state->decel_mrad = SM_STEPS_TO_MRAD(eep_params.sm_decel_steps);
	_state->speed_max_mrad = SM_STEPS_TO_MRAD(eep_params.sm_speed_max_steps);
}

void sm_enable(uint8_t enable)
{
	taskENTER_CRITICAL();

	if (enable)
	{
		ioport_set_pin_dir(_state->pin_enable, IOPORT_DIR_OUTPUT);
		ioport_set_pin_mode(_state->pin_enable, 0);
		ioport_set_pin_level(_state->pin_enable, false);
	}
	else
	{
		ioport_set_pin_dir(_state->pin_enable, IOPORT_DIR_INPUT);
		ioport_set_pin_mode(_state->pin_enable, IOPORT_MODE_PULLUP);
		ioport_set_pin_level(_state->pin_enable, true);
	}

	taskEXIT_CRITICAL();
}

void sm_enable_from_isr(uint8_t enable)
{
	taskDISABLE_INTERRUPTS();

	if (enable)
	{
		ioport_set_pin_dir(_state->pin_enable, IOPORT_DIR_OUTPUT);
		ioport_set_pin_mode(_state->pin_enable, 0);
		ioport_set_pin_level(_state->pin_enable, false);
	}
	else
	{
		ioport_set_pin_dir(_state->pin_enable, IOPORT_DIR_INPUT);
		ioport_set_pin_mode(_state->pin_enable, IOPORT_MODE_PULLUP);
		ioport_set_pin_level(_state->pin_enable, true);
	}

	taskENABLE_INTERRUPTS();
}

void sm_set_params(uint16_t accel, uint16_t decel, uint16_t speed_max,
		uint16_t reverse, uint16_t power_save)
{
	eep_params.sm_accel_steps = accel;
	eep_params.sm_decel_steps = decel;
	eep_params.sm_speed_max_steps = speed_max;
	eep_params.sm_motor_reverse = reverse;
	eep_params.sm_power_save = (uint8_t) power_save;
	eep_save();

	_state->accel_mrad = SM_STEPS_TO_MRAD(eep_params.sm_accel_steps);
	_state->decel_mrad = SM_STEPS_TO_MRAD(eep_params.sm_decel_steps);
	_state->speed_max_mrad = SM_STEPS_TO_MRAD(eep_params.sm_speed_max_steps);
}

void sm_get_params(uint16_t *accel, uint16_t *decel, uint16_t *speed_max,
		uint16_t *reverse, uint16_t *power_save)
{
	*accel = eep_params.sm_accel_steps;
	*decel = eep_params.sm_decel_steps;
	*speed_max = eep_params.sm_speed_max_steps;
	*reverse = eep_params.sm_motor_reverse;
	*power_save = eep_params.sm_power_save;
}

uint8_t sm_start(uint8_t dir)
{
	if (dir == CW)
	{
		return sm_move(INT32_MAX);
	}
	else
	{
		return sm_move(INT32_MIN + 1);
	}
}

uint8_t sm_move(int32_t step)
{
	return sm_move_ex(step, _state->speed_max_mrad, _state->accel_mrad, _state->decel_mrad);
}

uint8_t sm_move_ex(int32_t step, uint16_t speed, uint16_t accel, uint16_t decel)
{
	if (sm_get_state() != SM_STATE_STOP)
	{
		return 1;
	}

	SEGGER_RTT_printf(0, "move: %d %u %u %u\n", step, speed, accel, decel);

	int32_t max_s_lim;
	int32_t accel_lim;
	
	_state->remaining_steps = step;

	if (step < 0)
	{
		_state->dir = CCW;
		step = -step;
	}
	else
	{
		_state->dir = CW;
	}

	// If moving only 1 step.
	if (step == 1)
	{
		// Move one step...
		_state->accel_decel_step_count = -1;
		// ...in SM_STATE_DECEL state.
		_state->run_state = SM_STATE_DECEL;
		// Just a short delay so main() can act on 'running'.
		_state->step_delay = 1000;

		if (eep_params.sm_power_save == 1)
		{
			sm_enable(1);
		}

		// Run Timer/Counter.
		tc_write_rc(TC0, 0, 10);
		tc_start(TC0, 0);
	}
	// Only move if number of steps to move is not zero.
	else if (step != 0)
	{
		// Set max speed limit, by calc min_delay to use in timer.
		// min_delay = (alpha / tt) / w
		_state->min_step_delay = A_T_x1000 / speed;

		// Set acceleration by calc the first step delay.
		uint64_t step_delay_tmp = (T_FREQ_148 * l_sqrt(A_SQ / accel)) / 100;
		if (step_delay_tmp > INT32_MAX)
		{
			step_delay_tmp = INT32_MAX;
		}
		_state->step_delay = step_delay_tmp;

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
			_state->accel_val = accel_lim;
			_state->decel_val = (int32_t) accel_lim - step;
		}
		else
		{
			_state->accel_val = max_s_lim;
			_state->decel_val = -((int64_t) max_s_lim * (int64_t) accel) / (int64_t) decel;
		}
		// We must decelerate at least 1 step to stop.
		if (_state->decel_val == 0)
		{
			_state->decel_val = -1;
		}

		// Find step to start deceleration.
		_state->decel_start = step + _state->decel_val;

		// If the maximum speed is so low that we don't need to go via acceleration state.
		if (_state->step_delay <= _state->min_step_delay)
		{
			_state->step_delay = _state->min_step_delay;
			_state->run_state = SM_STATE_RUN;
		}
		else
		{
			_state->run_state = SM_STATE_ACCEL;
		}

		// Reset counter.
		_state->accel_decel_step_count = 0;

		SEGGER_RTT_printf(0, "move start: %u %u %d %d %d %u\n", _state->min_step_delay, _state->step_delay, _state->accel_decel_step_count, _state->accel_val, _state->decel_val, _state->decel_start);

		if (eep_params.sm_power_save == 1)
		{
			sm_enable(1);
		}

		// Set Timer/Counter.
		tc_write_rc(TC0, 0, 10);
		tc_start(TC0, 0);
	}

	return 0;
}

void sm_stop(void)
{
	taskENTER_CRITICAL();

	switch (_state->run_state)
	{
	case SM_STATE_STOP:
	case SM_STATE_DECEL:
		return;
	case SM_STATE_ACCEL:
		_state->accel_decel_step_count = -1 * (int64_t) _state->accel_decel_step_count * (int64_t) _state->accel_mrad / (int64_t) _state->decel_mrad;
		_state->run_state = SM_STATE_DECEL;
		break;
	case SM_STATE_RUN:
		_state->accel_decel_step_count = _state->decel_val;
		_state->run_state = SM_STATE_DECEL;
		break;
	}

	taskEXIT_CRITICAL();
}

void sm_emergency_stop(void)
{
	taskENTER_CRITICAL();
	_state->run_state = SM_STATE_STOP;
	taskEXIT_CRITICAL();
}

int32_t sm_get_remaining_steps(void)
{
	int32_t v;
	
	taskENTER_CRITICAL();
	v = _state->remaining_steps;
	taskEXIT_CRITICAL();

	return v;
}

int32_t sm_get_position(void)
{
	int32_t v;

	taskENTER_CRITICAL();
	v = _state->position;
	taskEXIT_CRITICAL();

	return v;
}

void sm_reset_position(void)
{
	taskENTER_CRITICAL();
	_state->position = 0;
	taskEXIT_CRITICAL();
}

uint8_t sm_get_state(void)
{
	uint8_t v;

	taskENTER_CRITICAL();
	v = _state->run_state;
	taskEXIT_CRITICAL();

	return v;
}

static void sm_step(uint8_t dir)
{
	if (dir == CW)
	{
		_state->position++;
		_state->remaining_steps--;
		if (!eep_params.sm_motor_reverse)
		{
			ioport_set_pin_level(_state->pin_dir, true);
		}
		else
		{
			ioport_set_pin_level(_state->pin_dir, false);
		}
	}
	else
	{
		_state->position--;
		_state->remaining_steps++;
		if (!eep_params.sm_motor_reverse)
		{
			ioport_set_pin_level(_state->pin_dir, false);
		}
		else
		{
			ioport_set_pin_level(_state->pin_dir, true);
		}
	}

	delay_us(1);
	ioport_set_pin_level(_state->pin_step, true);
	delay_us(2);
	ioport_set_pin_level(_state->pin_step, false);
	delay_us(1);
}

/* Square root routine.
 *
 * sqrt routine 'grupe', from comp.sys.ibm.pc.programmer
 * Subject: Summary: SQRT(int) algorithm (with profiling)
 *    From: warwick@cs.uq.oz.au (Warwick Allison)
 *    Date: Tue Oct 8 09:16:35 1991
 */
static uint32_t l_sqrt(uint32_t x)
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

/*  Timer/Counter1 Output Compare A Match Interrupt.
 *  Increments/decrements the position of the stepper motor
 *  except after last position, when it stops.
 *  The step_delay defines the period of this interrupt
 *  and controls the speed of the stepper motor.
 *  A new step delay is calculated to follow wanted speed profile
 *  on basis of accel/decel parameters.
 */
void TC0_Handler(void)
{
	// Remember the last step delay used when accelerating.
	static uint32_t last_accel_delay;
	// Counting steps when moving.
	static uint32_t step_count = 0;
	// Keep track of remainder from new_step_delay calculation to increase accuracy
	static int32_t rest = 0;

	volatile uint32_t dummy;
	dummy = tc_get_status(TC0, 0);
	UNUSED(dummy);

	// Holds next delay period.
	uint32_t new_step_delay = 0;

	tc_write_rc(TC0, 0, _state->step_delay);

	switch (_state->run_state)
	{
	case SM_STATE_STOP:
		_state->remaining_steps = 0;
		step_count = 0;
		rest = 0;
		if (eep_params.sm_power_save == 1)
		{
			sm_enable_from_isr(0);
		}
		tc_stop(TC0, 0);
		break;
	case SM_STATE_ACCEL:
		sm_step(_state->dir);
		step_count++;
		_state->accel_decel_step_count++;
		new_step_delay = (int32_t) _state->step_delay - (2 * (int32_t) _state->step_delay + rest) / (4 * _state->accel_decel_step_count + 1);
		if (new_step_delay > INT32_MAX)
		{
			new_step_delay = INT32_MAX;
		}
		rest = ((2 * (int32_t) _state->step_delay) + rest) % (4 * _state->accel_decel_step_count + 1);
		// Check if we should start deceleration.
		if (step_count >= _state->decel_start)
		{
			_state->accel_decel_step_count = _state->decel_val;
			_state->run_state = SM_STATE_DECEL;
		}
		// Check if we hit max speed.
		else if (new_step_delay <= _state->min_step_delay)
		{
			last_accel_delay = new_step_delay;
			new_step_delay = _state->min_step_delay;
			rest = 0;
			_state->run_state = SM_STATE_RUN;
		}
		break;
	case SM_STATE_RUN:
		sm_step(_state->dir);
		step_count++;
		new_step_delay = _state->min_step_delay;
		// Check if we should start deceleration.
		if (step_count >= _state->decel_start)
		{
			_state->accel_decel_step_count = _state->decel_val;
			// Start deceleration with same delay as accel ended with.
			new_step_delay = last_accel_delay;
			_state->run_state = SM_STATE_DECEL;
		}
		break;
	case SM_STATE_DECEL:
		sm_step(_state->dir);
		step_count++;
		_state->accel_decel_step_count++;
		new_step_delay = (int32_t) _state->step_delay - (2 * (int32_t) _state->step_delay + rest) / (4 * _state->accel_decel_step_count + 1);
		if (new_step_delay > INT32_MAX)
		{
			new_step_delay = INT32_MAX;
		}
		rest = (2 * (int32_t) _state->step_delay + rest) % (4 * _state->accel_decel_step_count + 1);
		// Check if we at last step
		if (_state->accel_decel_step_count >= 0)
		{
			new_step_delay = 10;
			_state->run_state = SM_STATE_STOP;
		}
		break;
	}

	_state->step_delay = new_step_delay;

	//SEGGER_RTT_printf(0, "move timer: %u %u %u %d %d %d %u\r\n", step_count, _state->min_step_delay, _state->step_delay, _state->accel_decel_step_count, _state->accel_val, _state->decel_val, _state->decel_start);
	//SEGGER_RTT_printf(0, "move timer: %u %u\r\n", step_count, _state->step_delay);
}
