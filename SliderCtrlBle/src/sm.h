#ifndef SM_H_
#define SM_H_

#include <stdint.h>
#include <math.h>

#define SM_RPS_TO_MRAD(r)	((uint32_t)(r)*2*M_PI*1000)
#define SM_RPM_TO_MRAD(r)	((uint32_t)(r)*2*M_PI*1000/60)
#define SM_STEPS_TO_MRAD(s) (2*M_PI*1000*(uint32_t)(s)/SM_SPR)
#define SM_MRAD_TO_STEPS(p) ((uint32_t)(p)*SM_SPR/(2*M_PI*1000))

#define SM_MAX_ACCEL_STEPS	(10*SM_SPR)
#define SM_MAX_SPEED_STEPS	(6*SM_SPR)

#define SM_STEPS_DIVISOR    16
#define SM_FULL_SPR         200
#define SM_SPR              (SM_FULL_SPR*SM_STEPS_DIVISOR)

#define SM_CW  1
#define SM_CCW 0

#define SM_STATE_STOP  0
#define SM_STATE_ACCEL 1
#define SM_STATE_DECEL 2
#define SM_STATE_RUN   3

typedef struct
{
	int32_t position;
	int32_t remaining_steps;
	uint16_t accel_mrad;
	uint16_t decel_mrad;
	uint16_t speed_max_mrad;
	
	uint8_t pin_enable;
	uint8_t pin_step;
	uint8_t pin_dir;
	
	uint8_t run_state:3;
	uint8_t dir:1;
	uint32_t step_delay;
	uint32_t min_step_delay;
	int32_t accel_val;
	int32_t decel_val;
	uint32_t decel_start;
	int32_t accel_decel_step_count;
} sm_state_t;

void sm_init(sm_state_t *state);
void sm_reload(void);
void sm_enable(uint8_t enable);
void sm_set_params(uint16_t accel, uint16_t decel, uint16_t speed_max, uint16_t reverse, uint16_t power_save);
void sm_get_params(uint16_t *accel, uint16_t *decel, uint16_t *speed_max, uint16_t *reverse, uint16_t *power_save);
uint8_t sm_move(int32_t step);
uint8_t sm_move_ex(int32_t step, uint16_t speed, uint16_t accel, uint16_t decel);
uint8_t sm_start(uint8_t dir);
void sm_stop(void);
void sm_emergency_stop(void);
void sm_reset_position(void);
int32_t sm_get_position(void);
uint8_t sm_get_state(void);
int32_t sm_get_remaining_steps(void);

#endif
