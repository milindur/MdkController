#ifndef EEP_H_
#define EEP_H_

#include <stdint.h>
#include "sm.h"
#include "mode_sms.h"

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
} eep_mode_sms_position_params_t;

typedef struct {
    uint16_t version;
    uint16_t check;
    
    char ble_device_name[20];

    eep_sm_params_t sm[SM_MOTORS_USED];
    
    uint32_t mode_sms_pre_time;
    uint32_t mode_sms_focus_time;
    uint32_t mode_sms_exposure_time;
    uint32_t mode_sms_post_time;
    
    uint8_t mode_sms_optimize_accel;
    uint8_t mode_sms_use_slider_as_shutter;
    
    uint32_t mode_sms_interval;
    uint32_t mode_sms_count;

    uint32_t mode_sms_leadin_count[SM_MOTORS_USED];
    uint32_t mode_sms_accel_count[SM_MOTORS_USED];
    uint32_t mode_sms_decel_count[SM_MOTORS_USED];
    uint32_t mode_sms_leadout_count[SM_MOTORS_USED];
    eep_mode_sms_position_params_t mode_sms_positions[MODE_SMS_MAX_KEY_FRAMES];
    
    uint32_t mode_video_duration[SM_MOTORS_USED];
    uint8_t mode_video_ping_pong;

    uint32_t mode_pano_pause;
    uint32_t mode_pano_count;
    
    eep_mode_sms_position_params_t mode_pano_position_start;
    eep_mode_sms_position_params_t mode_pano_position_stop;
    eep_mode_sms_position_params_t mode_pano_position_ref_start;
    eep_mode_sms_position_params_t mode_pano_position_ref_stop;
} eep_params_t;

extern eep_params_t eep_params;

void vEepInit(void);
void vEepSave(void);
void vEepLoad(void);
void vEepLoadDefault(void);

#endif
