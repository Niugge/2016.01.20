#ifndef _QUAD_TYPES_H_
#define _QUAD_TYPES_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"


/*---------------------* 
*   加速度计数据结构   * 
*----------------------*/
typedef struct 
{
	int16_t gx;
	int16_t gy;
	int16_t gz;
	int16_t ax;
	int16_t ay;
	int16_t az;
}MPU6050_DATA;

/*---------------------* 
*   HMC5883 数据类型   * 
*----------------------*/
typedef struct
{
	int16_t  hx;
	int16_t  hy;
	int16_t  hz;   
}HMC5883L_DATA;

/*---------------------*
*  遥控器              *
*----------------------*/
typedef struct 
{
	int16_t ROLL;
	int16_t PITCH;
	int16_t THROTTLE;
	int16_t YAW;
	int16_t AUX1;
	int16_t AUX2;
	int16_t AUX3;
	int16_t AUX4;
	int16_t AUX5;
	int16_t AUX6;
}T_RC_Data;

/*---------------------*
* GPS            *
*----------------------*/
typedef struct 
{
	vs32 LAT;
	vs32 LNG;
	float GPS_ALT;
	float GPS_SPD;
	int16_t GPS_HAC;
	int16_t GPS_VAC;
	u8 GPS_STA;
	u8 GPS_SVN;
}GPS_Data;
/*---------------------*
*  PWM                 *
*----------------------*/
typedef struct 
{
	int16_t MOTO1_PWM;
	int16_t MOTO2_PWM;
	int16_t MOTO3_PWM;
	int16_t MOTO4_PWM;
}MOTO_PWM;

typedef struct 
{
	float ROL_P;
	float ROL_I;
	float ROL_D;	
	float PIT_P;
	float PIT_I;
	float PIT_D;	
	float YAW_P;
	float YAW_I;
	float YAW_D;
	float ALT_P;
	float ALT_I;
	float ALT_D;
	float POS_P;
	float POS_I;
	float POS_D;
	float PID1_P;
	float PID1_I;
	float PID1_D;
}PID;

typedef struct 
{
	float ROL_RATE_P;
	float ROL_RATE_I;
	float ROL_RATE_D;	
	float PIT_RATE_P;
	float PIT_RATE_I;
	float PIT_RATE_D;	
	float YAW_RATE_P;
	float YAW_RATE_I;
	float YAW_RATE_D;
	float ALT_RATE_P;
	float ALT_RATE_I;
	float ALT_RATE_D;
	float POS_RATE_P;
	float POS_RATE_I;
	float POS_RATE_D;
	float PID1_RATE_P;
	float PID1_RATE_I;
	float PID1_RATE_D;
}PID_RATE;

typedef struct 
{
	float kp;
	float ki;
	float kd;
	float rate_kp;
	float rate_ki;
	float rate_kd;
	float outP;
	float outI;
	float outD;	
	float error_angle;
	float angle_I;
	float angle_P_term;
	float angle_I_Term;
	
	float angleRate_Ref;		
	float error_angleRate;
	float pre_error_angleRate;	
	float angelrate_delta;	
	float deriv_BUF[5];
	uint8_t filter_cnt;	
	float angleRate_I;
	
	float angleRate_P_Term;
	float angleRate_I_Term;
	float angleRate_D_Term;	
	
}STAND_PID;


typedef struct 
{
	float roll;
	float pitch;
	float yaw;
}ANGEL;

typedef struct
{
    float  hx;
    float  hy;
    float  hz;   
}HMC5883L_DATA_Unit;

typedef struct 
{
	float gx;
	float gy;
	float gz;
	float ax;
	float ay;
	float az;
}MPU6050_DATA_Unit;

typedef struct 
{
	float ax;
	float ay;
	float az;
}POS_RATE;

typedef struct 
{
	short ROL;
	short PIT;
	short YAW;
	short ALT_CSB;
	short ALT_PRS;
	uint8_t ARMED;
}STATUS_DATA;


//-----系统标志-----//
typedef struct
{
	unsigned char ALT_HOLD_MODE:1;		// 高度保持模式标志

}SYSTEM_STATE;

#endif
