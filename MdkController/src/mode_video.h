/*
 * mode_video.h
 *
 * Created: 06.04.2015 13:47:22
 *  Author: Christian
 */ 


#ifndef MODE_VIDEO_H_
#define MODE_VIDEO_H_

#define MODE_VIDEO_MAX_KEY_FRAMES			2

#define MODE_VIDEO_MAX_TIME		 			999999
#define MODE_VIDEO_MAX_TIME_HM              3599940
#define MODE_VIDEO_MAX_COUNT				30000
#define MODE_VIDEO_MAX_RAMP_COUNT			300
#define MODE_VIDEO_MAX_STALL_COUNT			300

#define MODE_VIDEO_STATE_STOP				0
#define MODE_VIDEO_STATE_WAKE_SM            1
#define MODE_VIDEO_STATE_GOTO_START         2
#define MODE_VIDEO_STATE_WAIT_START         3
#define MODE_VIDEO_STATE_MOVE		        4
#define MODE_VIDEO_STATE_WAIT_MOVE		    5
#define MODE_VIDEO_STATE_GOTO_END			6
#define MODE_VIDEO_STATE_WAIT_END			7
#define MODE_VIDEO_STATE_SLEEP_SM           8

#define MODE_VIDEO_STATE_MOVE_WAIT_LEAD_IN      0
#define MODE_VIDEO_STATE_MOVE_RUN               1
#define MODE_VIDEO_STATE_MOVE_WAIT_RUN          2
#define MODE_VIDEO_STATE_MOVE_WAIT_LEAD_OUT     3
#define MODE_VIDEO_STATE_MOVE_DONE              4

void vModeVideoInit(void);
void vModeVideoSetStartEnd(int32_t start, int32_t end);
void vModeVideoGetStartEnd(int32_t *start, int32_t *end);
void vModeVideoStart(void);
void vModeVideoPause(void);
void vModeVideoResume(void);
void vModeVideoStop(void);
uint8_t ucModeVideoGetState(void);
bool bModeVideoIsFinished(void);
bool bModeVideoIsPaused(void);
bool bModeVideoIsRunning(void);
uint8_t ucModeVideoGetProgress(void);
uint32_t ulModeVideoGetCurrentTime(void);
uint32_t ulModeVideoGetRemainingTime(void);
uint32_t ulModeVideoGetOverallTime(void);
void vModeVideoCalcTime(void);

#endif /* MODE_VIDEO_H_ */