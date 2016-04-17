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
bool bBleUpdateMoCoControlPointTx(uint8_t state, uint8_t * buffer, uint8_t length);
bool bBleProcessMoCoControlPointRx(uint8_t subadr, uint8_t cmd, uint8_t * data, uint8_t data_length);
bool bBleProcessMoCoControlPointRxMain(uint8_t cmd, uint8_t * data, uint8_t data_length);
bool bBleProcessMoCoControlPointRxMotor(uint8_t motor, uint8_t cmd, uint8_t * data, uint8_t data_length);
bool bBleProcessMoCoControlPointRxCamera(uint8_t cmd, uint8_t * data, uint8_t data_length);
bool bBleUpdateMoCoControlPointTxOkData(uint8_t * data, uint8_t length);
void vBleUpdateMoCoControlPointTxOk(void);
void vBleUpdateMoCoControlPointTxError(void);
void vBleSetJoystickWatchdog(uint8_t enable);
uint8_t ucBleGetJoystickWatchdog(void);
void vBleJoystickTriggerReset(void);
uint8_t ucBleGetMode(void);
void vBleSetMode(uint8_t value);

#endif /* BLE_H_ */
