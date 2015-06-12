/*
 * mode_sms.h
 *
 * Created: 06.04.2015 13:47:22
 *  Author: Christian
 */ 


#ifndef MODE_SMS_H_
#define MODE_SMS_H_

#define MODE_SMS_MAX_KEY_FRAMES			3

#define MODE_SMS_MAX_TIME		 			999999
#define MODE_SMS_MAX_TIME_HM              3599940
#define MODE_SMS_MAX_COUNT				30000
#define MODE_SMS_MAX_RAMP_COUNT			300
#define MODE_SMS_MAX_STALL_COUNT			300

#define MODE_SMS_STATE_STOP				0
#define MODE_SMS_STATE_WAKE_SM            1
#define MODE_SMS_STATE_GOTO_START         2
#define MODE_SMS_STATE_WAIT_START         3
#define MODE_SMS_STATE_WAIT_PRE_TIME		4
#define MODE_SMS_STATE_WAIT_FOCUS_TIME	5
#define MODE_SMS_STATE_WAIT_EXPOSURE_TIME 6
#define MODE_SMS_STATE_WAIT_POST_TIME		7
#define MODE_SMS_STATE_MOVE		        8
#define MODE_SMS_STATE_WAIT_MOVE		    9
#define MODE_SMS_STATE_WAIT_INTERVAL		10
#define MODE_SMS_STATE_GOTO_END			11
#define MODE_SMS_STATE_WAIT_END			12
#define MODE_SMS_STATE_SLEEP_SM           13

typedef struct {
	uint32_t pre_time;
	uint32_t focus_time;
	uint32_t exposure_time;
	uint32_t post_time;
} mode_sms_setup_t;

void vModeSmsInit(void);
void vModeSmsSetParams(mode_sms_setup_t *params);
void vModeSmsGetParams(mode_sms_setup_t *params);
void vModeSmsSetStartEnd(int32_t start, int32_t end);
void vModeSmsGetStartEnd(int32_t *start, int32_t *end);
void vModeSmsSetInterval(uint32_t interval, uint32_t count, uint16_t ramp_count, uint16_t stall_count);
void vModeSmsGetInterval(uint32_t *interval, uint32_t *count, uint16_t *ramp_count, uint16_t *stall_count);
void vModeSmsUpdateIntervalToMinimum(void);
uint32_t ulModeSmsGetMinimumInterval(uint32_t pre_time, uint32_t focus_time, uint32_t exposure_time, uint32_t post_time);
void vModeSmsStart(void);
void vModeSmsStartCameraTest(void);
void vModeSmsStartExposeNow(void);
void vModeSmsPause(void);
void vModeSmsResume(void);
void vModeSmsStop(void);
bool bModeSmsGetCameraTestMode(void);
uint8_t ucModeSmsGetState(void);
bool bModeSmsIsFinished(void);
bool bModeSmsIsPaused(void);
bool bModeSmsIsRunning(void);
uint32_t ulModeSmsGetCurrentCycle(void);
uint32_t ulModeSmsGetRemainingCycles(void);
uint32_t ulModeSmsGetOverallCycles(void);
uint8_t ucModeSmsGetProgress(void);
uint32_t ulModeSmsGetRemainingStepTime(void);
uint32_t ulModeSmsGetRemainingIntervalTime(void);
uint32_t ulModeSmsGetCurrentTime(void);
uint32_t ulModeSmsGetRemainingTime(void);
uint32_t ulModeSmsGetOverallTime(void);
void vModeSmsCalcTime(void);

#endif /* MODE_SMS_H_ */