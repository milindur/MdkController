/*
 * ble.h
 *
 * Created: 06.04.2015 13:48:00
 *  Author: Christian
 */ 


#ifndef BLE_H_
#define BLE_H_

typedef struct
{
	uint32_t ulElapsedCycles;
	uint32_t ulElapsedTime;
	uint32_t ulElapsedIntervalTime;
	uint32_t ulElapsedStepTime;
	uint32_t ulStepTime;
} BleSliderCycle_t;

typedef struct
{
	uint32_t ulProgramCycles;
	uint32_t ulProgramTime;
	uint32_t ulProgramIntervalTime;
} BleSliderProgram_t;

void vBleInit(void);
bool bBleUpdateSliderStateTx(uint8_t slider_state, uint8_t motor1_state, uint8_t motor2_state, uint8_t motor3_state, uint8_t motor4_state);
bool bBleUpdateSliderPositionTx(int32_t motor1_position, int32_t motor2_position, int32_t motor3_position, int32_t motor4_position);
bool bBleUpdateSliderCycleTx(BleSliderCycle_t * cycle);
bool bBleSetSliderProgram(BleSliderProgram_t * program);

#endif /* BLE_H_ */