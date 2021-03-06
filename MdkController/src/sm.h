#ifndef SM_H_
#define SM_H_

#include <stdint.h>
#include <math.h>

#define SM_MODE_STEPS_1         1
#define SM_MODE_STEPS_2         2
#define SM_MODE_STEPS_4         4
#define SM_MODE_STEPS_16        16
#define SM_MODE_STEPS_MASK      0x1F
#define SM_MODE_INTERPOLATION   0x40
#define SM_MODE_STEALTH         0x80

#define SM_MODE_STEPS_DEFAULT   SM_MODE_STEPS_16

#define SM_STEPS_DIVISOR    SM_MODE_STEPS_DEFAULT
#define SM_FULL_SPR         200
#define SM_SPR              (SM_FULL_SPR*SM_STEPS_DIVISOR)

#define SM_RPS_TO_MRAD(r)   ((uint32_t)(r)*2*M_PI*1000)
#define SM_RPM_TO_MRAD(r)   ((uint32_t)(r)*2*M_PI*1000/60)
#define SM_STEPS_TO_MRAD(s) (2*M_PI*1000*(uint32_t)(s)/SM_SPR)
#define SM_MRAD_TO_STEPS(p) ((uint32_t)(p)*SM_SPR/(2*M_PI*1000))

#define SM_MAX_ACCEL_STEPS          (10*SM_SPR)
#define SM_MAX_SPEED_STEPS          (6*SM_SPR)
#define SM_MAX_STEALTH_SPEED_STEPS  (4*SM_SPR)

#define SM_GEAR_REDUCTION_MDKv5		(60.0*(19.0/30.0)*(5.0+2.0/11.0))
#define SM_GEAR_REDUCTION_MDKv6		(60.0)
#define SM_GEAR_REDUCTION_NICOTILT	(32.0)

#define SM_ASTRO_SIDEREAL_FACTOR    (0.99726958)
#define SM_ASTRO_LUNAR_FACTOR       (1.03387012)

#define SM_MOTOR(m)			(1<<m)
#define SM_MOTOR_0			SM_MOTOR(0)
#define SM_MOTOR_1			SM_MOTOR(1)
#define SM_MOTOR_2			SM_MOTOR(2)
#define SM_MOTOR_ALL		(SM_MOTOR_0|SM_MOTOR_1|SM_MOTOR_2)
#define SM_MOTORS_USED      3

#define SM_CW               1
#define SM_CCW              0

#define SM_ASTRO_SIDEREAL   0
#define SM_ASTRO_LUNAR      1

#define SM_STATE_STOP       0
#define SM_STATE_ACCEL      1
#define SM_STATE_DECEL      2
#define SM_STATE_RUN        3
#define SM_STATE_CONT       4
#define SM_STATE_CONT_SLOW  5

void vSmInit(void);
void vSmSetMicrostepMode(uint8_t motor, uint8_t mode);
void vSmReload(void);
void vSmEnable(uint8_t motor, uint8_t enable);
void vSmSetParams(uint16_t accel, uint16_t decel, uint16_t speed_max, uint16_t reverse, uint16_t power_save);
void vSmGetParams(uint16_t *accel, uint16_t *decel, uint16_t *speed_max, uint16_t *reverse, uint16_t *power_save);
int32_t lSmGetMaxSpeed(uint8_t motor);
uint8_t ucSmMove(uint8_t motor, int32_t step);
uint8_t ucSmMoveEx(uint8_t motor, int32_t step, uint16_t speed, uint16_t accel, uint16_t decel);
bool bSmMoveContinuous(uint8_t motor, int32_t speed);
bool bSmMoveContinuousAstro(uint8_t motor, uint8_t dir, float_t gear_reduction, float_t factor);
void vSmStop(uint8_t motor);
void vSmEmergencyStop(uint8_t motor);
void vSmResetPosition(uint8_t motor);
int32_t lSmGetPosition(uint8_t motor);
uint8_t ucSmGetState(uint8_t motor);
int32_t lSmGetRemainingSteps(uint8_t motor);

#endif
