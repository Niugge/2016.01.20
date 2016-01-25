/**********************************************
 * @file global.h
 * @author leadger
 * @date 2015-10-14 18:42
 * @version 1.0
 **********************************************
 */

#include "global.h"


uint8_t GYRO_OFFSET_OK = 0;
uint8_t ACC_OFFSET_OK  = 0;
uint8_t CMP_OFFSET_OK  = 0;
uint8_t MS_OFFSET_OK   = 0;

/******�ɿ����ݴ洢********/
T_RC_Data 	  Rc_D;			//ң��ͨ������
HMC5883L_DATA hmc5883l;		//HMC5883Lԭʼ����
HMC5883L_DATA_Unit hmc5883l_cal;	//HMC5883��0��1����
MPU6050_DATA  mpu6050;		//MPU6050ԭʼ����
MPU6050_DATA  mpu6050_smooth;//MPU6050�˲�������
MPU6050_DATA  mpu6050_Butterworse;//MPU6050�˲�������
MPU6050_DATA_Unit  mpu6050_unit;
MPU6050_DATA_Unit  mpu6050_unit_deg;
POS_RATE Pos_rate;//λ���ٶ�
POS_RATE Pos_rate2;//λ���ٶ�
SYSTEM_STATE SYSTEM_STA; //ϵͳ����״̬
GPS_Data Gps_Data;       //gps����

ANGEL angel_quad;
ANGEL angel_quad1;

HMC5883L_DATA_Unit hmc5883l_unit;
uint8_t ARMed = 0;
vs32 BaroAlt = 0;			//��ѹ������
int16_t Calculate_THR; //�����������

PID      PID1_ipt;
PID_RATE PID2_ipt;
STAND_PID pid;
MOTO_PWM moto_rate;

uint8_t Send_PID1 = 0;

vs32 BaroOffset = 0;
vs32 Alt_Estimated=0;
vs32 Alt_Estimated2=0;
s8 test; 	 		//IIC�����õ�

float error_detal;
int32_t sum_filter_x,sum_filter_y; //�����������ĺ�������
u8 ADNS_cnt=0;

MPU6050_DATA MPU6050_OFFSER;
HMC5883L_DATA hmc_offset;

uint8_t sum_cnt = 0;
vs32 sum_temp = 0;
int16_t aoutpu_alt=0;

//uint16_t ANGEL_OR_RATE_CONTROL
float bettary = 0;
uint16_t cal_cnt;
float My_Gain,Mz_Gain;
vs16 mx_min,mx_max,my_min,my_max,mz_min,mz_max;
/***test data***/
MPU6050_DATA_Unit PI_dcm;
MPU6050_DATA_Unit non_dcm;

/* End of files ------------------------------------------------------*/
