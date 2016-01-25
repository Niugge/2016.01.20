#ifndef _ALTITUDE_
#define _ALTITUDE_

#include "MPU6050.H"
#include "HMC5883L.h"
#include "ms5611.h"
#include "tim_pwm_in.h"
#include "math.h"
#include "led.h"
#include "sys.h"
#include "HMC5883L.h"
#include "global.h"

void Altitude_Calculation(vs32 alt,MPU6050_DATA_Unit mpu6050_unit2, ANGEL angle_quad2,POS_RATE *PositionRate,vs32 *height);
void Altitude_Control(int16_t *outputPID);
float LPButter_Vel_Error(float curr_input);
void Kalman_Filter_Altitude(vs32 alt,MPU6050_DATA_Unit mpu6050_unit2, ANGEL angle_quad2,POS_RATE *PositionRate,vs32 *height); //¿¨¶ûÂüº¯Êý;
void Kalman_Comple_Filter_Altitude(vs32 alt,MPU6050_DATA_Unit mpu6050_unit2, ANGEL angle_quad2,POS_RATE *PositionRate,vs32 *height); //¿¨¶ûÂü»¥²¹º¯Êý;

#endif
