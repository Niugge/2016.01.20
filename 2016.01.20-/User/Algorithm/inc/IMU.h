#ifndef __IMU_H__
#define __IMU_H__

#include "MPU6050.H"
#include "HMC5883L.h"
#include "ms5611.h"
#include "tim_pwm_in.h"
#include "math.h"
#include "led.h"
#include "sys.h"
#include "HMC5883L.h"
#include "global.h"

void MPU6050_CalOff_Acc(void);
void MPU6050_CalOff_Gyr(void);
void Cal_Compass(void);
void MS5611_CalOffset(void );

void ACC_Smooth(MPU6050_DATA *acc_in,MPU6050_DATA *acc_out);
void Unit_Unify(MPU6050_DATA *mpu6050_in,HMC5883L_DATA_Unit *hmc5883l_in,MPU6050_DATA_Unit *mpu6050_out,MPU6050_DATA_Unit *mpu6050_deg_out,HMC5883L_DATA_Unit *hmc5883l_out);

void Rc_Fun(T_RC_Data *rc_in,uint8_t *armed);
float InvSqrt(float x);
void Rc_Fun(T_RC_Data *rc_in,uint8_t *armed);
void Kalman_Filter_X(float Accel,float Gyro,ANGEL *angel_out1); //¿¨¶ûÂüº¯Êý	
void Kalman_Filter_Y(float Accel,float Gyro,ANGEL *angel_out2); //¿¨¶ûÂüº¯Êý		
void Kalman_IMU_Update(MPU6050_DATA_Unit mpu6050_unit1,ANGEL *angel_out);
void Kalman_Grad_Update(MPU6050_DATA_Unit mpu6050_unit1,ANGEL *angel_out);

//#ifdef USE_IMU#endif
void IMU_Update(MPU6050_DATA_Unit mpu6050_unit1,ANGEL *angel_out);

#ifdef USE_Kalman_IMU
void Kalman_IMU_Update(MPU6050_DATA_Unit mpu6050_unit1,ANGEL *angel_out);
#endif
#ifdef USE_AHRS
void AHRSupdate(MPU6050_DATA_Unit mpu6050_unit,HMC5883L_DATA_Unit hmc5883_unit,ANGEL *angel_out) ;
#endif
#ifdef USE_GraDescent
void GraDescent_Updata(MPU6050_DATA_Unit mpu6050_unit1,ANGEL *angel_out);
#endif
#ifdef USE_GraDescentAll
void GraDescentAll_Updata(MPU6050_DATA_Unit mpu6050_unit1,HMC5883L_DATA_Unit hmc5883_unit,ANGEL *angel_out);
#endif
#ifdef USE_Kalman_Grad_ALL_IMU
void Kalman_Grad_ALL_Update(MPU6050_DATA_Unit mpu6050_unit1,HMC5883L_DATA_Unit hmc5883_unit,ANGEL *angel_out);
#endif

#endif



