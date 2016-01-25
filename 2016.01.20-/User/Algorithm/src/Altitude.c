#include "Altitude.h"

#define ALT_LIMIT_MIN   10
#define g_Earth 9.7936f     //地心引力
#define dT      T_cyc   //积分周期
#define DeadBand_Down_THR  1430   //定高油门死区
#define DeadBand_Up_THR  1600
#define Scale_Up_THR     0.94f    //超过定高油门缩放比例

///********************************************
//函数名:	void Altitude_Calculation(vs32 alt,MPU6050_DATA_Unit mpu6050_unit2, ANGEL angle_quad2,POS_RATE *PositionRate,vs32 *height)
//说明:	高度估计,Z轴速度估计
//入口:气压计原始高度，加速计原始数据，融合的角度，
//出口:	Z轴速度，高度
//备注:	无
//*********************************************/
void Altitude_Calculation(vs32 alt,MPU6050_DATA_Unit mpu6050_unit2, ANGEL angle_quad2,POS_RATE *PositionRate,vs32 *height)
{	
	float Altitude_error=0,az_Earth=0;
	static float Altitude_Estimated=0,Altitude_delt_error=0;//Velocity=0,
  static float Altitude_pre_error=0;
	float Baro_height=0;
	static float acc_z=0;
	
//  mpu6050_unit2.ax=mpu6050_unit2.ax*100;
//	mpu6050_unit2.ay=mpu6050_unit2.ay*100;
//	mpu6050_unit2.az=mpu6050_unit2.az*100;
	/******加速度计与气压计的两次一阶互补滤波*********/
  angle_quad2.pitch=angle_quad2.pitch/57.3;
	angle_quad2.roll=angle_quad2.roll/57.3;
	
	az_Earth=-sin(angle_quad2.pitch)*mpu6050_unit2.ax+cos(angle_quad2.pitch)*sin(angle_quad2.roll)*mpu6050_unit2.ay+cos(angle_quad2.pitch)*cos(angle_quad2.roll)*mpu6050_unit2.az-g_Earth;
	
	Altitude_error=alt/100.0f;
	if(Altitude_error!=Altitude_pre_error)
		Altitude_delt_error=Altitude_error-Altitude_pre_error;
	Altitude_pre_error=Altitude_error;
	
	acc_z=0.985*(acc_z+az_Earth*dT)+0.015*Altitude_delt_error; /*第一次一阶互补*/
//	
  Baro_height=alt/100.0f;
	Altitude_Estimated=0.985*(Altitude_Estimated+acc_z*dT)+0.015*Baro_height;/*第二次一阶互补*/
	*height=(vs32)(Altitude_Estimated*100.0);  //高度
	
	PositionRate->az=acc_z*100.0f; //速度
}

/******************************************************
函数名：void Kalman_Filter_Altitude(vs32 alt,MPU6050_DATA_Unit mpu6050_unit2, ANGEL angle_quad2,POS_RATE *PositionRate,vs32 *height) //卡尔曼函数	
说明：计算高度和z轴速度
入口：气压计高度，加速度计单位化的值，姿态角
出口：z轴速度，估计高度
备注：
*******************************************************/

//卡尔曼参数		
float Q_Altitude = 0.001f;  
float Q_Velocity  = 0.003f;
float R_Barometer = 0.5f;
//float dt      = 0.01;//dt为kalman滤波器采样时间;
char  C_0     = 1;
float vz_Earth=0;
float Q_bias,Q_bias_h, Altitude_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };
float Pdot_h[4] ={0,0,0,0};
float PP_h[2][2] = { { 1, 0 },{ 0, 1 } };

void Kalman_Comple_Filter_Altitude(vs32 alt,MPU6050_DATA_Unit mpu6050_unit2, ANGEL angle_quad2,POS_RATE *PositionRate,vs32 *height) //卡尔曼函数		
{
	float az_Earth=0;
	static float hz=0;
	float Altitude_error=0;
	//static float Altitude_Estimated=0;//Velocity=0,
  static float Altitude_pre_error=0,Altitude_delt_error=0;
//	float Baro_height=0;
	
  angle_quad2.pitch=angle_quad2.pitch/57.3;
	angle_quad2.roll=angle_quad2.roll/57.3;
	
	az_Earth = -sin(angle_quad2.pitch)*mpu6050_unit2.ax+cos(angle_quad2.pitch)*sin(angle_quad2.roll)*mpu6050_unit2.ay+cos(angle_quad2.pitch)*cos(angle_quad2.roll)*mpu6050_unit2.az-g_Earth;

//	hz += (vz_Earth - Q_bias) * dT;//先验估计
	Altitude_error=(float)(alt)/100.0f;
	if(Altitude_error!=Altitude_pre_error)
		Altitude_delt_error=Altitude_error-Altitude_pre_error;
	Altitude_pre_error=Altitude_error;
	
	vz_Earth=0.985*(vz_Earth+az_Earth*dT)+0.015*Altitude_delt_error; /*一阶互补*/
//	
	hz += (vz_Earth - Q_bias) * dT;//先验估计                       //现在状态，第一个公式
	
	Pdot[0]=Q_Altitude - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]= Q_Velocity;
	
	PP[0][0] += Pdot[0] * dT;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dT;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dT;
	PP[1][1] += Pdot[3] * dT;
		
	Altitude_err = alt/100.0f - hz;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];                                       //协方差，第二个公式
	PCt_1 = C_0 * PP[1][0];
	
	E = R_Barometer + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;                                              //卡尔曼增益，第三个公式
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差              //卡尔曼方差，第五个公式
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	hz     += K_0 * Altitude_err;	 //后验估计                        //卡尔曼数据，第四个公式
	Q_bias += K_1 * Altitude_err;	 //后验估计
	
	PositionRate->az    = (vz_Earth - Q_bias)*100.0f;	 //输出值(后验估计)的微分=角速度
	*height             =  hz*100;
	//Data_Send_DCM(0.0f,0.0f,Altitude_delt_error/dT,Pos_rate.az,BaroAlt,Alt_Estimated);
}

void Kalman_Filter_Altitude(vs32 alt,MPU6050_DATA_Unit mpu6050_unit2, ANGEL angle_quad2,POS_RATE *PositionRate,vs32 *height) //卡尔曼函数		
{
	float az_Earth=0;
	static float hz=0;
	float Altitude_error=0;
//	static float Altitude_Estimated=0;//Velocity=0,
  static float Altitude_pre_error=0,Altitude_delt_error=0;
//	float Baro_height=0;
	
  angle_quad2.pitch=angle_quad2.pitch/57.3;
	angle_quad2.roll=angle_quad2.roll/57.3;
	
	az_Earth = -sin(angle_quad2.pitch)*mpu6050_unit2.ax+cos(angle_quad2.pitch)*sin(angle_quad2.roll)*mpu6050_unit2.ay+cos(angle_quad2.pitch)*cos(angle_quad2.roll)*mpu6050_unit2.az-g_Earth;

//	hz += (vz_Earth - Q_bias) * dT;//先验估计
	Altitude_error=alt/100.0f;
	if(Altitude_error!=Altitude_pre_error)
		Altitude_delt_error=Altitude_error-Altitude_pre_error;
	Altitude_pre_error=Altitude_error;
	
	//vz_Earth=0.985*(vz_Earth+az_Earth*dT)+0.015*Altitude_delt_error; /*一阶互补*/
//	
	vz_Earth += (az_Earth - Q_bias) * dT;//先验估计                       //现在状态，第一个公式
	
	Pdot[0]=Q_Altitude - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]= Q_Velocity;
	
	PP[0][0] += Pdot[0] * dT;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dT;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dT;
	PP[1][1] += Pdot[3] * dT;
		
	Altitude_err = Altitude_delt_error - vz_Earth;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];                                       //协方差，第二个公式
	PCt_1 = C_0 * PP[1][0];
	
	E = R_Barometer + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;                                              //卡尔曼增益，第三个公式
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差              //卡尔曼方差，第五个公式
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	vz_Earth     += K_0 * Altitude_err;	 //后验估计                        //卡尔曼数据，第四个公式
	Q_bias += K_1 * Altitude_err;	 //后验估计
	
	PositionRate->az    = vz_Earth * 100.0f;	 //输出值(后验估计)的微分=角速度
	//*height             =  hz*100;
	
	hz += (vz_Earth - Q_bias_h) * dT;//先验估计                       //现在状态，第一个公式
	
	Pdot_h[0]=Q_Altitude - PP_h[0][1] - PP_h[1][0]; // Pk-先验估计误差协方差的微分

	Pdot_h[1]= -PP_h[1][1];
	Pdot_h[2]= -PP_h[1][1];
	Pdot_h[3]= Q_Velocity;
	
	PP_h[0][0] += Pdot_h[0] * dT;   // Pk-先验估计误差协方差微分的积分
	PP_h[0][1] += Pdot_h[1] * dT;   // =先验估计误差协方差
	PP_h[1][0] += Pdot_h[2] * dT;
	PP_h[1][1] += Pdot_h[3] * dT;
		
	Altitude_err = alt/100.0f - hz;	//zk-先验估计
	
	PCt_0 = C_0 * PP_h[0][0];                                       //协方差，第二个公式
	PCt_1 = C_0 * PP_h[1][0];
	
	E = R_Barometer + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;                                               //卡尔曼增益，第三个公式
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP_h[0][1];

	PP_h[0][0] -= K_0 * t_0;		 //后验估计误差协方差              //卡尔曼方差，第五个公式
	PP_h[0][1] -= K_0 * t_1;
	PP_h[1][0] -= K_1 * t_0;
	PP_h[1][1] -= K_1 * t_1;
		
	hz     += K_0 * Altitude_err;	 //后验估计                        //卡尔曼数据，第四个公式
	Q_bias_h += K_1 * Altitude_err;	 //后验估计
	
	//PositionRate->az    = (vz_Earth - Q_bias_h)*100.0f;	 //输出值(后验估计)的微分=角速度
	*height             =  hz*100;
}

/********************************************
函数名:	void Altitude_Control(int16_t *outputPID)
说明:	高度PID计算
入口: 无
出口:	基础油门
备注:	无
*********************************************/

void Altitude_Control(int16_t *outputPID)
{
	static int16_t BaseTHR=0;
	static float Altitude_Ref=0,Altitude_I=0,AltRate_I=0,pre_error_AltRate=0;
	float Alt_error=0,Altitude_P_term=0,Altitude_I_term=0,AltRate_Ref=0;
	float error_AltRate=0,delta_error_AltRate=0,AltRate_P_Term=0,AltRate_I_Term=0,AltRate_D_Term=0,output=0;

	static u8 BaseTHR_FLAG=1;
	//static STAND_Alt_PID *pid;
	//u8 ALT_HOLD_MODE=0;
	//
	if(ARMed && SYSTEM_STA.ALT_HOLD_MODE && Rc_D.THROTTLE>DeadBand_Down_THR && Rc_D.THROTTLE<DeadBand_Up_THR)  //&& Alt_Estimated>ALT_LIMIT_MIN
	{
		if(Altitude_Ref==0)
		{
			Altitude_Ref=Alt_Estimated;
			if(BaseTHR_FLAG)
			{
				BaseTHR=Rc_D.THROTTLE;
				BaseTHR_FLAG=0;
			}
		}
#ifdef ALTITUDE_CONTROL
		/*************外环高度************************/
		Alt_error=Altitude_Ref-Alt_Estimated;
		
		Altitude_P_term=PID1_ipt.ALT_P*Alt_error;
		
		Altitude_I     += Alt_error;
		Altitude_I      = Constrain_float(Altitude_I, -7000.0f, 7000.0f); //具体多少根据实际
		Altitude_I_term = PID1_ipt.ALT_I*Altitude_I;
		
		AltRate_Ref=Altitude_P_term+Altitude_I_term;
		/***********************************************/
#endif
		/*************内环z轴速度***********************/
		error_AltRate=AltRate_Ref-Pos_rate.az;
		
		error_AltRate=LPButter_Vel_Error(error_AltRate); //速度差巴特沃斯滤波
		//error_detal=error_AltRate;
		AltRate_P_Term=PID1_ipt.POS_P*error_AltRate;
		
		AltRate_I      += error_AltRate;
		AltRate_I       = Constrain_float(AltRate_I, -7000.0f, 7000.0f); //具体多少根据实际
		AltRate_I_Term  = PID1_ipt.POS_I*AltRate_I;
		
		delta_error_AltRate=error_AltRate-pre_error_AltRate;
		pre_error_AltRate=error_AltRate;
		AltRate_D_Term=PID1_ipt.POS_D*delta_error_AltRate;
		
		output=AltRate_P_Term+AltRate_I_Term+AltRate_D_Term;
		aoutpu_alt=output;
		*outputPID=BaseTHR+(int16_t)Constrain_float(output,-100.0f,100.0f);//具体多少根据实际
	}
	//
	else if(ARMed && SYSTEM_STA.ALT_HOLD_MODE && Rc_D.THROTTLE>=DeadBand_Up_THR) //超出定高死区，油门比例缩放
	{
		Altitude_I=0;
		AltRate_I=0;
		
		Altitude_Ref=0;
		*outputPID=Rc_D.THROTTLE*Scale_Up_THR;
	}
	else
	{
		Altitude_I=0;
		AltRate_I=0;
		
		Altitude_Ref=0;
		*outputPID=Rc_D.THROTTLE;
	}
	Data_Send_DCM(angel_quad.roll,angel_quad.pitch,angel_quad.yaw,Pos_rate.az,BaroAlt,Alt_Estimated);
	//Data_Send_DCM(mpu6050_smooth.ax,mpu6050_Butterworse.ax,mpu6050_smooth.ay,mpu6050_Butterworse.ay,mpu6050_smooth.az,mpu6050_Butterworse.az);
}

/* Low-pass ButterWorth filter */
/* Vel Error (PosControl) */
/* Order: 1 */
/* Cutoff Frequency: 2Hz */
const static float b_velEr[2] = {0.0245f, 0.0245f};
const static float a_velEr[2] = {1.0f, -0.9510f};
/*************************************************
函数名:	float LPButter_Vel_Error(float curr_input)
说明:	垂直速度误差低通滤波
入口:	float curr_input	当前输入加速度误差
出口:	无
备注:	1阶Butterworth低通滤波器
		Cutoff Fre. 2Hz
*************************************************/
float LPButter_Vel_Error(float curr_input)
{
	static float input[2];
	static float output[2];
	
	/* 获取最新x(n) */
	input[1] = curr_input;
	
	/* Butterworth滤波 */
	output[1] = b_velEr[0] * input[1] + b_velEr[1] * input[0] - a_velEr[1] * output[0];
	
	/* x(n) 序列保存 */
	input[0] = input[1];
	/* y(n) 序列保存 */
	output[0] = output[1];
	
	return output[1];
}
