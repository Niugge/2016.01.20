#ifndef _GLOBAL_H_
#define _GLOBAL_H_

#include "stm32f10x.h"

#define D_BYTE0(dwTemp)       (*(int16_t *)(&dwTemp))
#define D_BYTE1(dwTemp)       (*((int16_t *)(&dwTemp) + 1))

extern uint8_t GYRO_OFFSET_OK;
extern uint8_t ACC_OFFSET_OK;
extern uint8_t CMP_OFFSET_OK;
extern uint8_t MS_OFFSET_OK;	
/**���ܺ궨��****/
//#define USE_BATTERY_CHARGE
#define CONTROL_USE_RC
//#define CONTROL_USE_24L01
/******��С�������л����ܺ�********/
//#define USE_BIG_SIZE_BOARD
#define USE_LITTLE_SIZE_BOARD
//����ģʽѡ��(default��2401)
//#define DATA_TRANSFER_USE_USART
/**att contrlo mode****/
//#define att_control_two_loop
//#define use_att_control_singel_loop
/********��̬�ǿ��Ʒ�ʽѡ��*********/
#define ANGEL_CONTROL
//#define ANGEL_RATE_CONTROL
/********�߶ȿ��Ʒ�ʽѡ��*********/
#define ALTITUDE_CONTROL
//#define ALTITUDE_RATE_CONTROL
/**************��̬�����㷨***************/
#define USE_IMU
//#define USE_AHRS
//#define USE_GraDescent
//#define USE_GraDescentAll
//#define USE_Kalman_IMU
//#define USE_Kalman_Grad_IMU
//#define USE_Kalman_Grad_ALL_IMU
/**************�߶Ƚ����㷨***************/
#define USE_Complementary                         //���λ���
//#define USE_Kalman_Complementary                //һ�λ�����һ�ο�����
//#define USE_Kalman_                             //���ο�����
/**************������������**************/
#define T_cyc    0.006f
/******�ɿ����ݴ洢********/
extern T_RC_Data Rc_D;			//ң��ͨ������
extern HMC5883L_DATA hmc5883l;	//HMC5883Lԭʼ����
extern HMC5883L_DATA_Unit hmc5883l_cal;	//HMC5883��0��1����
extern MPU6050_DATA  mpu6050;	//MPU6050ԭʼ����
extern MPU6050_DATA  mpu6050_smooth;	//MPU6050�˲�������
extern MPU6050_DATA  mpu6050_Butterworse;//MPU6050�˲�������;
extern MPU6050_DATA_Unit  mpu6050_unit;
extern MPU6050_DATA_Unit  mpu6050_unit_deg;
extern POS_RATE Pos_rate;//λ���ٶ�
extern POS_RATE Pos_rate2;//λ���ٶ�
extern SYSTEM_STATE SYSTEM_STA; //ϵͳ����״̬
extern GPS_Data Gps_Data;       //gps����

extern ANGEL angel_quad;
extern ANGEL angel_quad1;
extern HMC5883L_DATA_Unit hmc5883l_unit;
extern uint8_t ARMed;
extern vs32  BaroAlt;
extern int16_t Calculate_THR; //�����������

extern PID PID1_ipt;
extern PID_RATE PID2_ipt;
extern STAND_PID pid;
extern MOTO_PWM moto_rate;

extern uint8_t Send_PID1;

extern float error_detal;
extern int32_t sum_filter_x,sum_filter_y; //�����������ĺ�������

extern vs32 BaroOffset;
extern vs32 Alt_Estimated;
extern vs32 Alt_Estimated2;
extern s8 test;//IIC�����õ�

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
