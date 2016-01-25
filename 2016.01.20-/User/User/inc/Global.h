#ifndef _GLOBAL_H_
#define _GLOBAL_H_

#include "stm32f10x.h"

#define D_BYTE0(dwTemp)       (*(int16_t *)(&dwTemp))
#define D_BYTE1(dwTemp)       (*((int16_t *)(&dwTemp) + 1))

extern uint8_t GYRO_OFFSET_OK;
extern uint8_t ACC_OFFSET_OK;
extern uint8_t CMP_OFFSET_OK;
extern uint8_t MS_OFFSET_OK;	
/**功能宏定义****/
//#define USE_BATTERY_CHARGE
#define CONTROL_USE_RC
//#define CONTROL_USE_24L01
/******大小开发板切换功能宏********/
//#define USE_BIG_SIZE_BOARD
#define USE_LITTLE_SIZE_BOARD
//发送模式选择(default：2401)
//#define DATA_TRANSFER_USE_USART
/**att contrlo mode****/
//#define att_control_two_loop
//#define use_att_control_singel_loop
/********姿态角控制方式选择*********/
#define ANGEL_CONTROL
//#define ANGEL_RATE_CONTROL
/********高度控制方式选择*********/
#define ALTITUDE_CONTROL
//#define ALTITUDE_RATE_CONTROL
/**************姿态解算算法***************/
#define USE_IMU
//#define USE_AHRS
//#define USE_GraDescent
//#define USE_GraDescentAll
//#define USE_Kalman_IMU
//#define USE_Kalman_Grad_IMU
//#define USE_Kalman_Grad_ALL_IMU
/**************高度解算算法***************/
#define USE_Complementary                         //两次互补
//#define USE_Kalman_Complementary                //一次互补，一次卡尔曼
//#define USE_Kalman_                             //两次卡尔曼
/**************代码运行周期**************/
#define T_cyc    0.006f
/******飞控数据存储********/
extern T_RC_Data Rc_D;			//遥控通道数据
extern HMC5883L_DATA hmc5883l;	//HMC5883L原始数据
extern HMC5883L_DATA_Unit hmc5883l_cal;	//HMC5883归0归1数据
extern MPU6050_DATA  mpu6050;	//MPU6050原始数据
extern MPU6050_DATA  mpu6050_smooth;	//MPU6050滤波后数据
extern MPU6050_DATA  mpu6050_Butterworse;//MPU6050滤波后数据;
extern MPU6050_DATA_Unit  mpu6050_unit;
extern MPU6050_DATA_Unit  mpu6050_unit_deg;
extern POS_RATE Pos_rate;//位置速度
extern POS_RATE Pos_rate2;//位置速度
extern SYSTEM_STATE SYSTEM_STA; //系统运行状态
extern GPS_Data Gps_Data;       //gps数据

extern ANGEL angel_quad;
extern ANGEL angel_quad1;
extern HMC5883L_DATA_Unit hmc5883l_unit;
extern uint8_t ARMed;
extern vs32  BaroAlt;
extern int16_t Calculate_THR; //计算出的油门

extern PID PID1_ipt;
extern PID_RATE PID2_ipt;
extern STAND_PID pid;
extern MOTO_PWM moto_rate;

extern uint8_t Send_PID1;

extern float error_detal;
extern int32_t sum_filter_x,sum_filter_y; //光流传感器的横纵坐标

extern vs32 BaroOffset;
extern vs32 Alt_Estimated;
extern vs32 Alt_Estimated2;
extern s8 test;//IIC计数用到

extern MPU6050_DATA MPU6050_OFFSER;
extern HMC5883L_DATA hmc_offset;
extern u8 ADNS_cnt;

extern uint8_t sum_cnt;
extern vs32 sum_temp;
extern int16_t aoutpu_alt;

extern float bettary ;
extern uint16_t cal_cnt;
extern float My_Gain,Mz_Gain;
extern vs16 mx_min,mx_max,my_min,my_max,mz_min,mz_max;

extern MPU6050_DATA_Unit PI_dcm;
extern MPU6050_DATA_Unit non_dcm;
#endif
