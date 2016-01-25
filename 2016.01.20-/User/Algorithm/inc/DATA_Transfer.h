#ifndef __DATA_TRANSFER_H
#define __DATA_TRANSFER_H
#include "Nrf24l01.h"
#include "HMC5883L.h"
#include "MPU6050.h"
#include "tim_pwm_in.h"
#include "ms5611.h"
#include "IMU.h"
#include "Moto.h"
#include "PID.h"
#include "led.h"
#include "sys.h"
#include "flash.h"
#include "eeprom.h"

void Nrf_Check_Event(void);
void Uart1_Put_Buf(uint8_t * tx_buf, uint8_t len);
void Data_Send_RCData(T_RC_Data *rc_opt);
void Data_Send_GPSData(GPS_Data *GPS_opt);
void Data_Send_IMU_Data(MPU6050_DATA *pt1,HMC5883L_DATA *pt2);
void Data_Send_MotoPWM(MOTO_PWM *moto_opt);
void Data_Send_Status(ANGEL *angel,vs32 *baro_alt,uint8_t armed);
void Data_Send_Check(uint16_t check);
void Data_Receive_Anl(uint8_t *data_buf,uint8_t num,T_RC_Data *rc_in,PID *pid1_in,PID_RATE *pid2_in);
void Data_Send_PID1(PID *pid1);
void Data_Send_PID2(PID *pid1);
void Data_Send_PID3(PID_RATE *pid2);
void Data_Send_PID4(PID_RATE *pid2);
void Data_Send_DCM(float a,float b,float c,float d,vs32 e ,vs32 f);
#endif

