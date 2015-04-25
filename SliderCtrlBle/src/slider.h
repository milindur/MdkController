/*
 * slider.h
 *
 * Created: 06.04.2015 13:47:22
 *  Author: Christian
 */ 


#ifndef SLIDER_H_
#define SLIDER_H_

#define SLIDER_MAX_KEY_FRAMES			3

#define SLIDER_MAX_TIME		 			999999
#define SLIDER_MAX_TIME_HM              3599940
#define SLIDER_MAX_COUNT				30000
#define SLIDER_MAX_RAMP_COUNT			300
#define SLIDER_MAX_STALL_COUNT			300

#define SLIDER_STATE_STOP				0
#define SLIDER_STATE_WAKE_SM            1
#define SLIDER_STATE_GOTO_START         2
#define SLIDER_STATE_WAIT_START         3
#define SLIDER_STATE_WAIT_PRE_TIME		4
#define SLIDER_STATE_WAIT_FOCUS_TIME	5
#define SLIDER_STATE_WAIT_EXPOSURE_TIME 6
#define SLIDER_STATE_WAIT_POST_TIME		7
#define SLIDER_STATE_MOVE		        8
#define SLIDER_STATE_WAIT_MOVE		    9
#define SLIDER_STATE_WAIT_INTERVAL		10
#define SLIDER_STATE_GOTO_END			11
#define SLIDER_STATE_WAIT_END			12
#define SLIDER_STATE_SLEEP_SM           13

typedef struct {
	uint32_t pre_time;
	uint32_t focus_time;
	uint32_t exposure_time;
	uint32_t post_time;
} slider_setup_t;

void vSliderInit(void);
void vSliderSetParams(slider_setup_t *params);
void vSliderGetParams(slider_setup_t *params);
void vSliderSetStartEnd(int32_t start, int32_t end);
void vSliderGetStartEnd(int32_t *start, int32_t *end);
void vSliderSetInterval(uint32_t interval, uint32_t count, uint16_t ramp_count, uint16_t stall_count);
void vSliderGetInterval(uint32_t *interval, uint32_t *count, uint16_t *ramp_count, uint16_t *stall_count);
void vSliderUpdateIntervalToMinimum(void);
uint32_t ulSliderGetMinimumInterval(uint32_t pre_time, uint32_t focus_time, uint32_t exposure_time, uint32_t post_time);
void vSliderStart(void);
void vSliderStop(void);
uint8_t ucSliderGetState(void);
uint32_t ulSliderGetCurrentCycle(void);
uint32_t ulSliderGetRemainingCycles(void);
uint32_t ulSliderGetOverallCycles(void);
uint8_t ucSliderGetProgress(void);
uint32_t ulSliderGetRemainingStepTime(void);
uint32_t ulSliderGetRemainingIntervalTime(void);
uint32_t ulSliderGetCurrentTime(void);
uint32_t ulSliderGetRemainingTime(void);
uint32_t ulSliderGetOverallTime(void);
void vSliderCalcTime(void);

#endif /* SLIDER_H_ */