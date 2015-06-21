/*
 * mode_pano.h
 *
 * Created: 10.06.2015 22:48:12
 *  Author: Christian
 */ 


#ifndef MODE_ASTRO_H_
#define MODE_ASTRO_H_

#define MODE_ASTRO_STATE_STOP				0
#define MODE_ASTRO_STATE_WAKE_SM			1
#define MODE_ASTRO_STATE_MOVE		        2
#define MODE_ASTRO_STATE_WAIT_MOVE		    3
#define MODE_ASTRO_STATE_SLEEP_SM           4

void vModeAstroInit(void);
void vModeAstroStart(void);
void vModeAstroPause(void);
void vModeAstroResume(void);
void vModeAstroStop(void);
uint8_t ucModeAstroGetState(void);
bool bModeAstroIsFinished(void);
bool bModeAstroIsPaused(void);
bool bModeAstroIsRunning(void);

#endif /* MODE_ASTRO_H_ */