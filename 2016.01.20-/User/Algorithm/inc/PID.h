#ifndef __PID_H
#define __PID_H
#include "moto.h"
#include "sys.h"
#include "IMU.h"
#include "DATA_Transfer.h"

void Angel_Control(ANGEL *att_in,MPU6050_DATA_Unit *gyr_in, T_RC_Data *rcer_in, uint8_t Armed);

int16_t Constrain_int(int16_t value, const int16_t limit_Low, const int16_t limit_High);

float Constrain_float(float value, float limit_Low, float limit_High);

int16_t double_loop_pid(STAND_PID *pid, const float measured, int16_t joystick,float gyro);

int16_t yaw_rate_pidUpdate(MPU6050_DATA_Unit *gyr_in, T_RC_Data *rcer_in);



#endif

