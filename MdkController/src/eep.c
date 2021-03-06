#include <string.h>

#include "asf.h"

#include "version.h"
#include "sm.h"
#include "eep.h"

#define EEP_VERSION 0x0115
#define EEP_CHECK   0xaa55

#define EEP_BASE_ADDRESS 0x000FFC00

const eep_params_t eep_params_def = {
    .version = EEP_VERSION,
    .check = EEP_CHECK, 
    .ble_device_name = versionDEVICE_NAME_DEFAULT,
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
    .mode_sms_post_time = 3200,
    .mode_sms_optimize_accel = 1,
    .mode_sms_use_slider_as_shutter = 0,
    .mode_sms_interval = 5000,
    .mode_sms_count = 121,
    .mode_sms_leadin_count = { 0, 0, 0 },
    .mode_sms_accel_count = { 0, 0, 0 },
    .mode_sms_decel_count = { 0, 0, 0 },
    .mode_sms_leadout_count = { 0, 0, 0 },
    .mode_sms_positions[0] = {
        .pos = { 0, 0, 0 }  
    },
    .mode_sms_positions[1] = {
        .pos = { 0, 0, 0 }
    },
    .mode_video_duration = { 10000, 10000, 10000 },
    .mode_video_ping_pong = 0,
    .mode_pano_pause = 10000,
    .mode_pano_count = 1,
    .mode_pano_position_start = {
        .pos = { 0, 0, 0 }
    },
    .mode_pano_position_stop = {
        .pos = { 0, 0, 0 }
    },
    .mode_pano_position_ref_start = {
        .pos = { 0, 0, 0 }
    },
    .mode_pano_position_ref_stop = {
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
    //nvm_read(INT_FLASH, EEP_BASE_ADDRESS + 0, (void *)&eep_params, sizeof(eep_params_t));
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
