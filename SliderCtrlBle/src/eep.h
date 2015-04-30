#ifndef EEP_H_
#define EEP_H_

#include <stdint.h>
#include "sm.h"
#include "slider.h"

typedef struct {
	uint8_t microstep_mode;
	uint16_t accel_steps;
	uint16_t decel_steps;
	uint16_t speed_max_steps;
	uint8_t power_save;
	uint8_t motor_reverse;
} eep_sm_params_t;

typedef struct {
	int32_t pos[SM_MOTORS_USED];
} eep_slider_position_params_t;

typedef struct {
	uint16_t version;
	uint16_t check;

	eep_sm_params_t sm[SM_MOTORS_USED];
	
	uint32_t slider_pre_time;
	uint32_t slider_focus_time;
	uint32_t slider_exposure_time;
	uint32_t slider_post_time;
	
	uint8_t slider_optimize_accel;
    
	uint32_t slider_interval;
    uint32_t slider_count;
    uint16_t slider_ramp_count;
    uint16_t slider_stall_count;
	
	eep_slider_position_params_t slider_positions[SLIDER_MAX_KEY_FRAMES];
} eep_params_t;

extern eep_params_t eep_params;

void vEepInit(void);
void vEepSave(void);
void vEepLoad(void);
void vEepLoadDefault(void);

#endif
