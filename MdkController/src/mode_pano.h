/*
 * mode_pano.h
 *
 * Created: 10.06.2015 22:48:12
 *  Author: Christian
 */ 


#ifndef MODE_PANO_H_
#define MODE_PANO_H_

#define MODE_PANO_STATE_STOP                0
#define MODE_PANO_STATE_WAKE_SM             1
#define MODE_PANO_STATE_GOTO_START          2
#define MODE_PANO_STATE_WAIT_START          3
#define MODE_PANO_STATE_WAIT_PRE_TIME       4
#define MODE_PANO_STATE_WAIT_FOCUS_TIME     5
#define MODE_PANO_STATE_WAIT_EXPOSURE_TIME  6
#define MODE_PANO_STATE_WAIT_POST_TIME      7
#define MODE_PANO_STATE_MOVE                8
#define MODE_PANO_STATE_WAIT_MOVE           9
#define MODE_PANO_STATE_WAIT_PAUSE          10
#define MODE_PANO_STATE_SLEEP_SM            11

void vModePanoInit(void);
void vModePanoStart(uint8_t motor_mask, bool allow_reversed_order);
void vModePanoPause(void);
void vModePanoResume(void);
void vModePanoStop(void);
uint8_t ucModePanoGetState(void);
bool bModePanoIsFinished(void);
bool bModePanoIsPaused(void);
bool bModePanoIsRunning(void);
uint32_t ulModePanoGetCurrentCycle(void);
uint32_t ulModePanoGetRemainingCycles(void);
uint32_t ulModePanoGetOverallCycles(void);
uint32_t ulModePanoGetCurrentRow(void);
uint32_t ulModePanoGetOverallRows(void);
uint32_t ulModePanoGetCurrentCol(void);
uint32_t ulModePanoGetOverallCols(void);
uint8_t ucModePanoGetProgress(void);

#endif /* MODE_PANO_H_ */