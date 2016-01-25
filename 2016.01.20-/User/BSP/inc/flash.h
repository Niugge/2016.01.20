#ifndef _FLASH_H_
#define _FLASH_H_

#include "stm32f10x.h"
#include "eeprom.h"
#include "MPU6050.h"
#include "PID.h"
#include "IMU.h"
#include "HMC5883L.h"



void EE_INIT(void);
void EE_SAVE_ACC_OFFSET(void);
void EE_READ_ACC_OFFSET(void);
void EE_SAVE_GYRO_OFFSET(void);
void EE_READ_GYRO_OFFSET(void);
void EE_SAVE_CMP_OFFSET(void);
void EE_READ_CMP_OFFSET(void);
void EE_SAVE_BARO_OFFSET(void);
void EE_READ_BARO_OFFSET(void);
void EE_SAVE_PID1(void);
void EE_SAVE_PID2(void);
void EE_SAVE_PID3(void);
void EE_SAVE_PID4(void);
void EE_READ_PID(void);

#endif
