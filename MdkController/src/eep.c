#include <string.h>

#include "asf.h"

#include "sm.h"
#include "eep.h"

#define EEP_VERSION 0x0101
#define EEP_CHECK   0xaa55

#define EEP_BASE_ADDRESS 0x000FFC00

const eep_params_t eep_params_def = {
	.version = EEP_VERSION,
	.check = EEP_CHECK,	
	.ble_device_name = "MDK Pan/Tilt",
	.sm[0] = {
		.microstep_mode = SM_MODE_INTERPOLATION | SM_MODE_STEALTH | SM_MODE_STEPS_DEFAULT,
		.accel_steps = 10*SM_SPR,
		.decel_steps = 10*SM_SPR,
		.speed_max_steps = 6*SM_SPR,
		.motor_reverse = 0,
		.power_save = 2
	},
	.sm[1] = {
		.microstep_mode = SM_MODE_INTERPOLATION | SM_MODE_STEALTH | SM_MODE_STEPS_DEFAULT,
		.accel_steps = 7*SM_SPR,
		.decel_steps = 7*SM_SPR,
		.speed_max_steps = 5*SM_SPR,
		.motor_reverse = 0,
		.power_save = 2
	},
	.sm[2] = {
		.microstep_mode = SM_MODE_INTERPOLATION | SM_MODE_STEALTH | SM_MODE_STEPS_DEFAULT,
		.accel_steps = 7*SM_SPR,
		.decel_steps = 7*SM_SPR,
		.speed_max_steps = 5*SM_SPR,
		.motor_reverse = 0,
		.power_save = 2
	},
	.mode_sms_pre_time = 0,
	.mode_sms_focus_time = 100,
	.mode_sms_exposure_time = 100,
	.mode_sms_post_time = 100,
	.mode_sms_optimize_accel = 1,
	.mode_sms_interval = 8000,
	.mode_sms_count = 100,
	.mode_sms_ramp_count = 0,
	.mode_sms_stall_count = 0,
	.mode_sms_positions[0] = {
		.pos = { 0, 0, 0 }	
	},
	.mode_sms_positions[1] = {
		.pos = { 0, 0, 0 }
	},
	.mode_sms_positions[2] = {
		.pos = { 0, 0, 0 }
	}
};
eep_params_t eep_params;

void vEepInit(void)
{
	if (nvm_init(INT_FLASH) != STATUS_OK)
	{
		for (;;) {}
	}
    vEepLoad();
}

void vEepSave(void)
{
	//nvm_write(INT_FLASH, EEP_BASE_ADDRESS + 0, &eep_params, sizeof(eep_params_t));
}

void vEepLoad(void)
{
	nvm_read(INT_FLASH, EEP_BASE_ADDRESS + 0, (void *)&eep_params, sizeof(eep_params_t));
	//if (eep_params.version != EEP_VERSION || eep_params.check != EEP_CHECK)
	{
		vEepLoadDefault();
		vEepSave();
	}
}

void vEepLoadDefault(void)
{
	memcpy(&eep_params, &eep_params_def, sizeof(eep_params_t));
}
