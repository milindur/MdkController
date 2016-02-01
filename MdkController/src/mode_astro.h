/*
 * mode_pano.h
 *
 * Created: 10.06.2015 22:48:12
 *  Author: Christian
 */ 


#ifndef MODE_ASTRO_H_
#define MODE_ASTRO_H_

#define MODE_ASTRO_DIR_NORTH                0
#define MODE_ASTRO_DIR_SOUTH                1

#define MODE_ASTRO_SPEED_SIDEREAL           0
#define MODE_ASTRO_SPEED_LUNAR              1

#define MODE_ASTRO_STATE_STOP               0
#define MODE_ASTRO_STATE_WAKE_SM            1
#define MODE_ASTRO_STATE_BACKLASH_1         2
#define MODE_ASTRO_STATE_WAIT_BACKLASH_1    3
#define MODE_ASTRO_STATE_BACKLASH_2         4
#define MODE_ASTRO_STATE_WAIT_BACKLASH_2    5
#define MODE_ASTRO_STATE_MOVE               6
#define MODE_ASTRO_STATE_WAIT_MOVE          7
#define MODE_ASTRO_STATE_SLEEP_SM           8

void vModeAstroInit(void);
void vModeAstroStart(uint8_t dir, uint8_t spd);
void vModeAstroPause(void);
void vModeAstroResume(void);
void vModeAstroStop(void);
uint8_t ucModeAstroGetState(void);
bool bModeAstroIsFinished(void);
bool bModeAstroIsPaused(void);
bool bModeAstroIsRunning(void);

#endif /* MODE_ASTRO_H_ */