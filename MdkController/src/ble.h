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
} BleModeSmsCycle_t;

typedef struct
{
    uint32_t ulProgramCycles;
    uint32_t ulProgramTime;
    uint32_t ulProgramIntervalTime;
} BleModeSmsProgram_t;

void vBleInit(void);

#endif /* BLE_H_ */