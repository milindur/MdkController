#ifndef EEP_H_
#define EEP_H_

#include <stdint.h>

typedef struct {
	uint16_t version;
	uint16_t check;
	uint16_t _reserved_00;
	uint16_t _reserved_01;
	uint8_t reset_cnt_jt;
	uint8_t reset_cnt_wd;
	uint8_t reset_cnt_bo;
	uint8_t reset_cnt_ext;
	uint8_t reset_cnt_po;
	uint16_t _reserved_02;
	uint16_t _reserved_03;
	uint16_t _reserved_04;
	uint16_t _reserved_05;
	uint16_t _reserved_06;
	uint16_t _reserved_07;
	uint16_t _reserved_08;
	uint16_t sm_accel_steps;
	uint16_t sm_decel_steps;
	uint16_t sm_speed_max_steps;
	uint8_t sm_power_save;
	uint8_t sm_motor_reverse;
	uint16_t _reserved_10;
	uint16_t _reserved_11;
	uint16_t _reserved_12;
	uint16_t _reserved_13;
	uint32_t slider_pre_time;
	uint32_t slider_focus_time;
	uint32_t slider_exposure_time;
	uint32_t slider_post_time;
    uint16_t _reserved_14;
    uint16_t _reserved_15;
    uint16_t _reserved_16;
    uint8_t _reserved_17;
	uint8_t slider_optimize_accel;
    uint32_t slider_interval;
    uint32_t slider_count;
    int32_t slider_start_position;
    int32_t slider_end_position;
    uint16_t slider_ramp_count;
    uint16_t slider_stall_count;
    uint16_t _reserved_18;
    uint16_t _reserved_19;
	uint8_t lcd_backlight_full_brightness;
	uint8_t lcd_backlight_reduced_brightness;
	uint16_t lcd_backlight_reduce_time;
	uint16_t lcd_backlight_off_time;
} eep_params_t;

extern eep_params_t eep_params;

void eep_init(void);
void eep_save(void);
void eep_load(void);
void eep_load_default(void);

#endif
