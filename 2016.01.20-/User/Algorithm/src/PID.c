#include "PID.h"
#define IMU_UPDATE_DT 0.004f
#define JOYSTICK_ZOOM_RATE  5u
#define RC_Conver_P 1400.0f   //遥控P转换比例
#define RC_Conver_I 28000.0f  //遥控I转换比例
/***************准备数据********
******1、飞机当前姿态角*********
******2、遥控器4通道输入********
3、陀螺仪去零漂后数据，替代微分环节
******4、遥控器解锁标记位*******
******5、电机转速占空比*********
******6、PID参数****************
******7、气压高度**厘米*********
********************************/
/********双环位置式PID**********/
void Angel_Control(ANGEL *att_in,MPU6050_DATA_Unit *gyr_in, T_RC_Data *rcer_in, uint8_t Armed)
{
	int16_t rol_out,pitch_out,yaw_out;
	static STAND_PID pid_roll,pid_pitch;  //,pid_yaw
	/**********PID init*********/
		pid_roll.kp      = PID1_ipt.ROL_P;
		pid_roll.ki      = PID1_ipt.ROL_I;
		pid_roll.kd      = PID1_ipt.ROL_D;	
		pid_roll.rate_kp = PID2_ipt.ROL_RATE_P;
		pid_roll.rate_ki = PID2_ipt.ROL_RATE_I;
		pid_roll.rate_kd = PID2_ipt.ROL_RATE_D;	
		/***************************/
		pid_pitch.kp      = PID1_ipt.PIT_P;
		pid_pitch.ki      = PID1_ipt.PIT_I;
		pid_pitch.kd      = PID1_ipt.PIT_D;
		pid_pitch.rate_kp = PID2_ipt.PIT_RATE_P;
		pid_pitch.rate_ki = PID2_ipt.PIT_RATE_I;
		pid_pitch.rate_kd = PID2_ipt.PIT_RATE_D;
		/****************************/	
//		pid_yaw.kp      = PID1_ipt.YAW_P;
//		pid_yaw.ki      = PID1_ipt.YAW_I;
//		pid_yaw.kd      = PID1_ipt.YAW_D;
//		pid_yaw.rate_kp = PID2_ipt.YAW_RATE_P;
//		pid_yaw.rate_ki = PID2_ipt.YAW_RATE_I;
//		pid_yaw.rate_kd = PID2_ipt.YAW_RATE_D;
		
	/****************************/	
	rol_out   = double_loop_pid(&pid_roll,  att_in->roll,  rcer_in->ROLL,  gyr_in->gx);	
	pitch_out = double_loop_pid(&pid_pitch, att_in->pitch, rcer_in->PITCH, gyr_in->gy);
	//yaw_out   = double_loop_pid(&pid_yaw,   att_in->yaw,   rcer_in->YAW,   gyr_in->gz);	
	yaw_out   = yaw_rate_pidUpdate(&mpu6050_unit_deg, &Rc_D);	
	/******test code******/
	///**neihuan**/
	//Data_Send_DCM(pid_roll.angleRate_Ref,gyr_in->gx,pid_roll.angleRate_P_Term,pid_roll.angleRate_I_Term,pid_roll.angleRate_D_Term ,rol_out);
	/**WAIHUAN**/
	//Data_Send_DCM(((rcer_in->THROTTLE - 1500)/30.0f),pid_roll.angleRate_Ref,gyr_in->gx,att_in->roll ,pid_roll.angle_I_Term,rol_out);
	/*********************/
	//Data_Send_DCM(angel_quad.pitch,angel_quad.roll,angel_quad1.pitch, angel_quad1.roll,angel_quad.yaw,-angel_quad1.yaw);
	/****************************/	
	if(Armed && rcer_in->THROTTLE > 1150)//(rcer_in->THROTTLE - 1500)/30.0f)
	{
		moto_rate.MOTO1_PWM = Calculate_THR - pitch_out - rol_out + yaw_out;
		moto_rate.MOTO2_PWM = Calculate_THR - pitch_out + rol_out - yaw_out;
		moto_rate.MOTO3_PWM = Calculate_THR + pitch_out + rol_out + yaw_out;
		moto_rate.MOTO4_PWM = Calculate_THR + pitch_out - rol_out - yaw_out;
	}
	else
	{
		moto_rate.MOTO1_PWM = 0;
		moto_rate.MOTO2_PWM = 0;
		moto_rate.MOTO3_PWM = 0;
		moto_rate.MOTO4_PWM = 0;
	}
		Moto_PwmRflash(&moto_rate);
}	

/*********双闭环，外环角度PI控制，内环角速度PID********/
int16_t double_loop_pid(STAND_PID *pid, const float measured,int16_t joystick,float gyro)
{	
	uint8_t i = 0;
	float temp;
	int16_t outputPID;
	/*********************************************************/
	//  Rc_D.AUX1<1500,角度外环控制，否则，角速度外环控制
	/**********************************************************/
#ifdef ANGEL_CONTROL//if(Rc_D.AUX1<1500)
{
	float expect;
	expect = (float)((joystick - 1500)/20.0f);
	pid->error_angle = expect - measured;
	/**********************************/
	pid->angle_P_term = pid->kp * pid->error_angle;	
	/* 飞机快要起飞时才引入积分,防止初始状态下就开始积分 */		
	if (Rc_D.THROTTLE > 1150)
	{
		pid->angle_I += pid->error_angle;
		pid->angle_I = Constrain_float(pid->angle_I, -PID2_ipt.POS_RATE_P*100.0f , PID2_ipt.POS_RATE_P*100.0f );
		pid->angle_I_Term = pid->ki * pid->angle_I;
	}
	else	// 积分清零
	{
		pid->angle_I = 0.0f;
		pid->angle_I_Term = 0.0f;
	}		
	pid->angleRate_Ref = pid->angle_P_term + pid->angle_I_Term;	
}
#endif
	/*********暴力模式*************/	
#ifdef ANGEL_RATE_CONTROL //else
	pid->angleRate_Ref = (float)((joystick - 1500)/16.384f);	 //角速度控制,单位为°/s*JOYSTICK_ZOOM_RATE
#endif	
	/************ 角速度内环PID ***************/
	pid->error_angleRate  = pid->angleRate_Ref - gyro;//角速度单位°/s	
	pid->angleRate_P_Term = pid->rate_kp * pid->error_angleRate;
	/***********************/
	if (Rc_D.THROTTLE > 1150)
	{
		pid->angleRate_I += pid->error_angleRate;
		pid->angleRate_I = Constrain_float(pid->angleRate_I,-(PID2_ipt.PID1_RATE_P*100.0f),PID2_ipt.PID1_RATE_P*100.0f); //10000.0f
		pid->angleRate_I_Term = pid->rate_ki * pid->angleRate_I;
	}
	else	// 积分清零
	{
		pid->angleRate_I = 0.0f;
		pid->angleRate_I_Term = 0.0f;
	}	
	/* 微分项滤波delta = 本次角速度误差 - 上次角速度误差 */
	pid->deriv_BUF[pid->filter_cnt] = (pid->error_angleRate - pid->pre_error_angleRate); 
	pid->pre_error_angleRate = pid->error_angleRate;
	for(i=0;i<5;i++)
	{
		temp += pid->deriv_BUF[i];
	}
	pid->angelrate_delta = temp / 5.0f;	
	
	pid->filter_cnt++;
	if(pid->filter_cnt == 5)	pid->filter_cnt = 0;
	pid->angleRate_D_Term = pid->rate_kd * pid->angelrate_delta;	
	/***************************************/
	outputPID = (int16_t)(pid->angleRate_P_Term + pid->angleRate_I_Term + pid->angleRate_D_Term);	
	return outputPID;
}   

int16_t yaw_rate_pidUpdate(MPU6050_DATA_Unit *gyr_in, T_RC_Data *rcer_in)
{
	int16_t output;
	float yaw_except;
	static STAND_PID pid_yaw;
	float yaw_error,yaw_delt_error;
	static float yaw_int,yaw_pre_error;	
	/********yaw角速度控制******/
	if(rcer_in->YAW >1520 || rcer_in->YAW<1480)
		yaw_except = (rcer_in->YAW - 1500)*5/16.384f; //角速度控制,单位为°/s
	else
		yaw_except = 0;	
	pid_yaw.kp = PID2_ipt.YAW_RATE_P;
	pid_yaw.ki = PID2_ipt.YAW_RATE_I;
	pid_yaw.kd = PID2_ipt.YAW_RATE_D;
	/****************************/
	yaw_error = (yaw_except - gyr_in->gz);//角速度误差
	pid_yaw.outP = pid_yaw.kp * yaw_error;	
	/****************************/
	if(rcer_in->THROTTLE > 1250)
	{
		yaw_int += yaw_error ;//积分
		yaw_int = Constrain_float(yaw_int, -3000.0f, 3000.0f);		
		pid_yaw.outI = pid_yaw.ki * yaw_int;	
	}
	else
	{
		yaw_int = 0.0f;
		pid_yaw.outI = 0.0f;
	}
	/*****************************/
	yaw_delt_error = yaw_error - yaw_pre_error;
	yaw_pre_error  = yaw_error;
	pid_yaw.outD   = pid_yaw.kd * yaw_delt_error;	
	/****************************/
	output = (int16_t)(pid_yaw.outP + pid_yaw.outI + pid_yaw.outD);	
	/****************************/
	return output;
}

float Constrain_float(float value, float limit_Low, float limit_High)
{
	if (value >= limit_High)
		value = limit_High;
	else if (value <= limit_Low)
		value = limit_Low;
	return value;
}
