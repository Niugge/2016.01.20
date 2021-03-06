#include "IMU.h"
#include "matrix.h"

#define Kp 2.5f       // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0012f        // integral gain governs rate of convergence of gyroscope biases
#define halfT T_cyc/2.0   // half the sample period,unit:second
#define Rad_To_Angle 	57.324841f
#define HEAD_LPF_FACTOR 20
#define sampleFreq  1.0f/T_cyc  //采样频率
#define g_Eart 9.7936f     //地心引力

//#define ddT 0.004f  //气压计使用

#define RC_FUN_MAX	1800
#define RC_FUN_MIN	1150
float  q0=1, q1=0, q2=0 , q3=0 ; //四元数初始化

/*****去零飘标记清零*******/
void MPU6050_CalOff_Acc(void)
{
	ACC_OFFSET_OK = 1;
	MPU6050_OFFSER.ax = 0;
	MPU6050_OFFSER.ay = 0;
	MPU6050_OFFSER.az = 0;
	yellow_OFF();
}

void MPU6050_CalOff_Gyr(void)
{
	GYRO_OFFSET_OK = 1;
	MPU6050_OFFSER.gx = 0;
	MPU6050_OFFSER.gy = 0;
	MPU6050_OFFSER.gz = 0;
	q0=1;q1=0;q2=0;q3=0 ;
	yellow_ON();
}

void Cal_Compass(void)
{
	CMP_OFFSET_OK = 1;
	cal_cnt = 15000;
	yellow_ON();
	mx_min=0;
	mx_max=0;
	my_min=0;
	my_max=0;
	mz_min=0;
	mz_max=0;
}

void MS5611_CalOffset(void )
{
	MS_OFFSET_OK = 1;
	BaroOffset = 0;
	sum_temp = 0;
	sum_cnt = BARO_CAL_CNT;
}

void Unit_Unify(MPU6050_DATA *mpu6050_in,HMC5883L_DATA_Unit *hmc5883l_in,MPU6050_DATA_Unit *mpu6050_out,MPU6050_DATA_Unit *mpu6050_deg_out,HMC5883L_DATA_Unit *hmc5883l_out)
{
	//将加速度计测量值转化成国际单位m/s^2,武汉的重力加速度为9.7936
	mpu6050_out->ax = (float)(mpu6050_in->ax/1672.93f);  //1671.78f对应+-2g
	mpu6050_out->ay = (float)(mpu6050_in->ay/1672.93f);  //835.918f对应+-4g
	mpu6050_out->az = (float)(mpu6050_in->az/1672.93f);
	//将陀螺仪测量值转换成国际单位 弧度/s
	mpu6050_out->gx = (float)(mpu6050_in->gx/939.68f); 
	mpu6050_out->gy = (float)(mpu6050_in->gy/939.68f);
	mpu6050_out->gz = (float)(mpu6050_in->gz/939.68f);
	//将陀螺仪测量值转到°/s方便角度控制使用
	mpu6050_deg_out->gx = (float)(mpu6050_in->gx/16.4f); 
	mpu6050_deg_out->gy = (float)(mpu6050_in->gy/16.4f);
	mpu6050_deg_out->gz = (float)(mpu6050_in->gz/16.4f);	
	//将磁强计测量值装换成国际单位
	hmc5883l_out->hx = (float) (hmc5883l_in->hx/1.0f);
	hmc5883l_out->hy = (float) (hmc5883l_in->hy/1.0f);
	hmc5883l_out->hz = (float) (hmc5883l_in->hz/1.0f);
}
/*******************************************************
1、坐标轴转动顺序Z->Y->X********************************
2、机头指向：+X轴***************************************
3、欧拉角定义：绕z-航向yaw，绕y-俯仰pitch，绕x-横滚roll*/
void IMU_Update(MPU6050_DATA_Unit mpu6050_unit1,ANGEL *angel_out)
{
	float norm1=0;
	float norm2=0;
	float vx=0, vy=0, vz=0;
	static float exInt = 0, eyInt = 0, ezInt = 0;  
	float ex = 0, ey = 0, ez = 0;	
//	float t11=0,t12=0;
	float t13=0,t23=0,t33=0;
	
	//ACC归一化，只剩下方向，没有大小
	norm1=InvSqrt(mpu6050_unit1.ax*mpu6050_unit1.ax + mpu6050_unit1.ay*mpu6050_unit1.ay + mpu6050_unit1.az*mpu6050_unit1.az);
	mpu6050_unit1.ax *= norm1;
	mpu6050_unit1.ay *= norm1;
	mpu6050_unit1.az *= norm1;
	//用四元数矩阵导出各轴重力分量
	vx=2*q1*q3 - 2*q0*q2;//vx=vx/g,unitlise
	vy=2*q0*q1 + 2*q2*q3;
	vz=(q0*q0  - q1*q1 - q2*q2 + q3*q3);
	//计算两坐标系之间误差  向量差积推导
	ex=(mpu6050_unit1.ay*vz - mpu6050_unit1.az*vy);
	ey=(mpu6050_unit1.az*vx - mpu6050_unit1.ax*vz);
	ez=(mpu6050_unit1.ax*vy - mpu6050_unit1.ay*vx);
	//误差I  求和
	exInt=exInt + ex*Ki;//PID1_ipt.PID1_I
	eyInt=eyInt + ey*Ki;//PID1_ipt.PID1_I
	ezInt=ezInt + ez*Ki;//PID1_ipt.PID1_I
	//将误差PI后补偿到陀螺仪，即补偿零点漂移	
	mpu6050_unit1.gx = mpu6050_unit1.gx+exInt + Kp*ex;//PID1_ipt.PID1_P
	mpu6050_unit1.gy = mpu6050_unit1.gy+eyInt + Kp*ey;//PID1_ipt.PID1_P
	mpu6050_unit1.gz = mpu6050_unit1.gz+ezInt + Kp*ez;//PID1_ipt.PID1_P
		
	//四元数微分方程的Rungkuta求解  更新四元数
	q0=q0+(-mpu6050_unit1.gx*q1 - mpu6050_unit1.gy*q2 - mpu6050_unit1.gz*q3)*halfT;
	q1=q1+( mpu6050_unit1.gx*q0 + mpu6050_unit1.gz*q2 - mpu6050_unit1.gy*q3)*halfT;
	q2=q2+( mpu6050_unit1.gy*q0 - mpu6050_unit1.gz*q1 + mpu6050_unit1.gx*q3)*halfT;
	q3=q3+( mpu6050_unit1.gz*q0 + mpu6050_unit1.gy*q1 - mpu6050_unit1.gx*q2)*halfT;
	//四元数的规范化
	norm2=InvSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 *= norm2;
	q1 *= norm2;
	q2 *= norm2;
	q3 *= norm2;
	//四元数方向余弦矩阵
//	t11=q0*q0 + q1*q1 - q2*q2 - q3*q3;
//	t12=2*q1*q2 + 2*q0*q3;
	t13=2*q1*q3 - 2*q0*q2;
	t23=2*q2*q3 + 2*q0*q1;
	t33=q0*q0 - q1*q1 - q2*q2 + q3*q3;
	//导出姿态角 GYRO
//	angel_out->yaw   =  atan2(t12,t11) * Rad_To_Angle;//罗盘计算角度替换
	angel_out->pitch = -asin(t13) * Rad_To_Angle ;
	angel_out->roll  =  atan2(t23,t33) * Rad_To_Angle;
}
/****************归0归1***************/
void Get_CompassAngle(HMC5883L_DATA_Unit *mag,float *heading)
{
	uint8_t i=0;
	vs32 sum = 0;	
	static uint8_t head_temp_cnt = 0;
	static vs16 head_temp[HEAD_LPF_FACTOR] = { 0 };
	float nx = 0,ny = 0;
		
	nx = cos(angel_quad.pitch/Rad_To_Angle) * mag->hx + sin(angel_quad.pitch/Rad_To_Angle)* sin(angel_quad.roll/Rad_To_Angle) * mag->hy + cos(angel_quad.roll/Rad_To_Angle) * sin(angel_quad.pitch/Rad_To_Angle) * mag->hz;
	ny = cos(angel_quad.roll/Rad_To_Angle) * mag->hy - sin(angel_quad.roll/Rad_To_Angle) * mag->hz;
	
	head_temp[head_temp_cnt++] = (vs16)(atan2(ny,nx)* Rad_To_Angle * 100);
	if(head_temp_cnt == HEAD_LPF_FACTOR)	
		head_temp_cnt = 0;
	
	for(i=0;i<HEAD_LPF_FACTOR;i++)
		sum += head_temp[i];
	*heading = -(float)((sum / HEAD_LPF_FACTOR)/100.0f);			
}


//=====================================================================================================
// Function  AHRS		Attitude Heading Reference System	姿态和方位参照系统
//===================================================================================================*/
void AHRSupdate(MPU6050_DATA_Unit mpu6050_unit1,HMC5883L_DATA_Unit hmc5883_unit,ANGEL *angel_out) 
{
        float norm;
		float t11=0,t12=0;
		float t13=0,t23=0,t33=0;
        float ex, ey, ez;	
        float hx, hy, hz, bx, bz;
        float vx, vy, vz, wx, wy, wz;
		static float exInt = 0, eyInt = 0, ezInt = 0; // scaled integral error
        /*auxiliary variables to reduce number of repeated operations*/ 
        float q0q0 = q0*q0;
        float q0q1 = q0*q1;
        float q0q2 = q0*q2;
        float q0q3 = q0*q3;
        float q1q1 = q1*q1;
        float q1q2 = q1*q2;
        float q1q3 = q1*q3;
        float q2q2 = q2*q2; 
        float q2q3 = q2*q3;
        float q3q3 = q3*q3;                 
        /*normalise the measurements*/ 
		norm=InvSqrt(mpu6050_unit1.ax*mpu6050_unit1.ax + mpu6050_unit1.ay*mpu6050_unit1.ay + mpu6050_unit1.az*mpu6050_unit1.az);
		mpu6050_unit1.ax *= norm;
		mpu6050_unit1.ay *= norm;
		mpu6050_unit1.az *= norm;
		
		norm = sqrt(hmc5883_unit.hx*hmc5883_unit.hx + hmc5883_unit.hy*hmc5883_unit.hy + hmc5883_unit.hz*hmc5883_unit.hz);          
		hmc5883_unit.hx = hmc5883_unit.hx / norm;
		hmc5883_unit.hy = hmc5883_unit.hy / norm;
		hmc5883_unit.hz = hmc5883_unit.hz / norm;                 
		/*compute reference direction of flux*/ 
		hx = 2*hmc5883_unit.hx*(0.5 - q2q2 - q3q3) + 2*hmc5883_unit.hy*(q1q2 - q0q3)       + 2*hmc5883_unit.hz*(q1q3 + q0q2);
		hy = 2*hmc5883_unit.hx*(q1q2 + q0q3)       + 2*hmc5883_unit.hy*(0.5 - q1q1 - q3q3) + 2*hmc5883_unit.hz*(q2q3 - q0q1);
		hz = 2*hmc5883_unit.hx*(q1q3 - q0q2)       + 2*hmc5883_unit.hy*(q2q3 + q0q1)       + 2*hmc5883_unit.hz*(0.5 - q1q1 - q2q2);         
		bx = sqrt((hx*hx) + (hy*hy));
		bz = hz;               
		/*estimated direction of gravity and flux (v and w)*/ 
		vx = 2*(q1q3 - q0q2);
		vy = 2*(q0q1 + q2q3);
		vz = q0q0 - q1q1 - q2q2 + q3q3;

		wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
		wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
		wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
		
		/*error is sum of cross product between reference direction of fields and direction measured by sensors*/ 
		ex = (mpu6050_unit1.ay*vz - mpu6050_unit1.az*vy) + (hmc5883_unit.hy*wz - hmc5883_unit.hz*wy);
		ey = (mpu6050_unit1.az*vx - mpu6050_unit1.ax*vz) + (hmc5883_unit.hz*wx - hmc5883_unit.hx*wz);
		ez = (mpu6050_unit1.ax*vy - mpu6050_unit1.ay*vx) + (hmc5883_unit.hx*wy - hmc5883_unit.hy*wx);
		
        /*integral error scaled integral gain*/ 
		exInt=exInt + ex*Ki;//PID1_ipt.PID1_I
		eyInt=eyInt + ey*Ki;//PID1_ipt.PID1_I
		ezInt=ezInt + ez*Ki;//PID1_ipt.PID1_I
        
        /*adjusted gyroscope measurements*/ 
		mpu6050_unit1.gx = mpu6050_unit1.gx+exInt + Kp*ex;//PID1_ipt.PID1_P
		mpu6050_unit1.gy = mpu6050_unit1.gy+eyInt + Kp*ey;//PID1_ipt.PID1_P
		mpu6050_unit1.gz = mpu6050_unit1.gz+ezInt + Kp*ez;//PID1_ipt.PID1_P        
        /*integrate quaternion rate and normalise*/ 
		q0=q0+(-mpu6050_unit1.gx*q1 - mpu6050_unit1.gy*q2 - mpu6050_unit1.gz*q3)*halfT;
		q1=q1+( mpu6050_unit1.gx*q0 + mpu6050_unit1.gz*q2 - mpu6050_unit1.gy*q3)*halfT;
		q2=q2+( mpu6050_unit1.gy*q0 - mpu6050_unit1.gz*q1 + mpu6050_unit1.gx*q3)*halfT;
		q3=q3+( mpu6050_unit1.gz*q0 + mpu6050_unit1.gy*q1 - mpu6050_unit1.gx*q2)*halfT;       
        /* normalise quaternion*/      
	    norm = InvSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 *= norm;
        q1 *= norm;
        q2 *= norm;
        q3 *= norm;
        /*auxiliary variables to reduce number of repeated operations*/ 
        q0q0 = q0*q0;
        q0q1 = q0*q1;
        q0q2 = q0*q2;
        q0q3 = q0*q3;
        q1q1 = q1*q1;
        q1q2 = q1*q2;
        q1q3 = q1*q3;
        q2q2 = q2*q2; 
        q2q3 = q2*q3;
        q3q3 = q3*q3; 		
		/*四元数方向余弦矩阵*/
		t11=q0q0 + q1q1 - q2q2 - q3q3;
		t12=2*q1q2 + 2*q0q3;
		t13=2*q1q3 - 2*q0q2;
		t23=2*q2q3 + 2*q0q1;
		t33=q0q0 - q1q1 - q2q2 + q3q3;
		/*导出姿态角 GYRO*/
		angel_out->yaw   =  atan2(t12,t11) * Rad_To_Angle;
		angel_out->pitch = -asin(t13) * Rad_To_Angle ;
		angel_out->roll  =  atan2(t23,t33) * Rad_To_Angle;						
}

/**************************************************************/
//
//		梯度下降法求姿态：陀螺仪、加速度计
//
/**************************************************************/
/* 梯度下降法的步长 */
	/*
		* 步长过短,静态性能增强,动态性能减弱
		* 步长过长,静态性能减弱,动态性能增强
		*一般范围在0.03~0.1左右
	*/
static float beta=0.02;

void GraDescent_Updata(MPU6050_DATA_Unit mpu6050_unit1,ANGEL *angel_out)
{
//	float recipNorm;
//	float s0, s1, s2, s3;
//	float t13=0,t23=0,t33=0;
//	float qDot1, qDot2, qDot3, qDot4;
//	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3, q0q1, q0q2, q1q3, q2q3;
//	float delta;								// 二阶毕卡法求解微分方程
//	
//	// Rate of change of quaternion from gyroscope
//	qDot1 = 0.5f * (-q1 * mpu6050_unit1.gx - q2 * mpu6050_unit1.gy - q3 * mpu6050_unit1.gz);
//	qDot2 = 0.5f * (q0 * mpu6050_unit1.gx + q2 * mpu6050_unit1.gz - q3 * mpu6050_unit1.gy);
//	qDot3 = 0.5f * (q0 * mpu6050_unit1.gy - q1 * mpu6050_unit1.gz + q3 * mpu6050_unit1.gx);
//	qDot4 = 0.5f * (q0 * mpu6050_unit1.gz + q1 * mpu6050_unit1.gy - q2 * mpu6050_unit1.gx);

//	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//	if(!((mpu6050_unit1.ax == 0.0f) && (mpu6050_unit1.ay == 0.0f) && (mpu6050_unit1.az == 0.0f))) {

//		// Normalise accelerometer measurement
//		recipNorm = InvSqrt(mpu6050_unit1.ax * mpu6050_unit1.ax + mpu6050_unit1.ay * mpu6050_unit1.ay + mpu6050_unit1.az * mpu6050_unit1.az);
//		mpu6050_unit1.ax *= recipNorm;
//		mpu6050_unit1.ay *= recipNorm;
//		mpu6050_unit1.az *= recipNorm;   

//		// Auxiliary variables to avoid repeated arithmetic
//		_2q0 = 2.0f * q0;
//		_2q1 = 2.0f * q1;
//		_2q2 = 2.0f * q2;
//		_2q3 = 2.0f * q3;
//		_4q0 = 4.0f * q0;
//		_4q1 = 4.0f * q1;
//		_4q2 = 4.0f * q2;
//		_8q1 = 8.0f * q1;
//		_8q2 = 8.0f * q2;
//		q0q0 = q0 * q0;
//		q1q1 = q1 * q1;
//		q2q2 = q2 * q2;
//		q3q3 = q3 * q3;
//		q0q1 = q0 * q1;
//		q0q2 = q0 * q2;
//		q1q3 = q1 * q3;
//		q2q3 = q2 * q3;

//		// Gradient decent algorithm corrective step
//		s0 = _4q0 * q2q2 + _2q2 * mpu6050_unit1.ax + _4q0 * q1q1 - _2q1 * mpu6050_unit1.ay;
//		s1 = _4q1 * q3q3 - _2q3 * mpu6050_unit1.ax + 4.0f * q0q0 * q1 - _2q0 * mpu6050_unit1.ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * mpu6050_unit1.az;
//		s2 = 4.0f * q0q0 * q2 + _2q0 * mpu6050_unit1.ax + _4q2 * q3q3 - _2q3 * mpu6050_unit1.ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * mpu6050_unit1.az;
//		s3 = 4.0f * q1q1 * q3 - _2q1 * mpu6050_unit1.ax + 4.0f * q2q2 * q3 - _2q2 * mpu6050_unit1.ay;
//		recipNorm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
//		s0 *= recipNorm;
//		s1 *= recipNorm;
//		s2 *= recipNorm;
//		s3 *= recipNorm;

//		// Apply feedback step
//		qDot1 -= beta * s0;
//		qDot2 -= beta * s1;
//		qDot3 -= beta * s2;
//		qDot4 -= beta * s3;
//	}

//	/* 将四元数姿态导数积分,得到当前四元数姿态 */
//	/* 二阶毕卡求解微分方程 */
//	delta = (T_cyc * mpu6050_unit1.gx) * (T_cyc * mpu6050_unit1.gx) + (T_cyc * mpu6050_unit1.gy) * (T_cyc * mpu6050_unit1.gy) +(T_cyc * mpu6050_unit1.gz) * (T_cyc * mpu6050_unit1.gz);
//	q0 = (1.0f - delta / 8.0f) * q0 + qDot1 * T_cyc;
//	q1 = (1.0f - delta / 8.0f) * q1 + qDot2 * T_cyc;
//	q2 = (1.0f - delta / 8.0f) * q2 + qDot3 * T_cyc;
//	q3 = (1.0f - delta / 8.0f) * q3 + qDot4 * T_cyc;

//	// Normalise quaternion
//	recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//	q0 *= recipNorm;
//	q1 *= recipNorm;
//	q2 *= recipNorm;
//	q3 *= recipNorm;
//	/*四元数方向余弦矩阵*/
//	t13=2*q1q3 - 2*q0q2;
//	t23=2*q2q3 + 2*q0q1;
//	t33=q0q0 - q1q1 - q2q2 + q3q3;
//	/*导出姿态角 GYRO*/
//	//angel_out->yaw   =  atan2(t12,t11) * Rad_To_Angle;
//	angel_out->pitch = -asin(t13) * Rad_To_Angle ;
//	angel_out->roll  =  atan2(t23,t33) * Rad_To_Angle;	
}

/**************************************************************/
//
//		梯度下降法求全姿态：陀螺仪、加速度计、磁强计
//
/**************************************************************/
void GraDescentAll_Updata(MPU6050_DATA_Unit mpu6050_unit1,HMC5883L_DATA_Unit hmc5883_unit,ANGEL *angel_out) 
{
//	float recipNorm;
//	float s0, s1, s2, s3;
//	float t11=0,t12=0,t13=0,t23=0,t33=0;
//	float qDot1, qDot2, qDot3, qDot4;
//	float hx, hy;
//	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
//	float delta;								// 二阶毕卡法求解微分方程
//	
//	// Rate of change of quaternion from gyroscope
//	qDot1 = 0.5f * (-q1 * mpu6050_unit1.gx - q2 * mpu6050_unit1.gy - q3 * mpu6050_unit1.gz);
//	qDot2 = 0.5f * (q0 * mpu6050_unit1.gx + q2 * mpu6050_unit1.gz - q3 * mpu6050_unit1.gy);
//	qDot3 = 0.5f * (q0 * mpu6050_unit1.gy - q1 * mpu6050_unit1.gz + q3 * mpu6050_unit1.gx);
//	qDot4 = 0.5f * (q0 * mpu6050_unit1.gz + q1 * mpu6050_unit1.gy - q2 * mpu6050_unit1.gx);

//	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
//	if(!((mpu6050_unit1.ax == 0.0f) && (mpu6050_unit1.ay == 0.0f) && (mpu6050_unit1.az == 0.0f))) {

//		// Normalise accelerometer measurement
//		recipNorm = InvSqrt(mpu6050_unit1.ax * mpu6050_unit1.ax + mpu6050_unit1.ay * mpu6050_unit1.ay + mpu6050_unit1.az * mpu6050_unit1.az);
//		mpu6050_unit1.ax *= recipNorm;
//		mpu6050_unit1.ay *= recipNorm;
//		mpu6050_unit1.az *= recipNorm;   

//		// Normalise magnetometer measurement
//		recipNorm = InvSqrt(hmc5883_unit.hx * hmc5883_unit.hx + hmc5883_unit.hy * hmc5883_unit.hy + hmc5883_unit.hz * hmc5883_unit.hz);
//		hmc5883_unit.hx *= recipNorm;
//		hmc5883_unit.hy *= recipNorm;
//		hmc5883_unit.hz *= recipNorm;

//		// Auxiliary variables to avoid repeated arithmetic
//		_2q0mx = 2.0f * q0 * hmc5883_unit.hx;
//		_2q0my = 2.0f * q0 * hmc5883_unit.hy;
//		_2q0mz = 2.0f * q0 * hmc5883_unit.hz;
//		_2q1mx = 2.0f * q1 * hmc5883_unit.hx;
//		_2q0 = 2.0f * q0;
//		_2q1 = 2.0f * q1;
//		_2q2 = 2.0f * q2;
//		_2q3 = 2.0f * q3;
//		_2q0q2 = 2.0f * q0 * q2;
//		_2q2q3 = 2.0f * q2 * q3;
//		q0q0 = q0 * q0;
//		q0q1 = q0 * q1;
//		q0q2 = q0 * q2;
//		q0q3 = q0 * q3;
//		q1q1 = q1 * q1;
//		q1q2 = q1 * q2;
//		q1q3 = q1 * q3;
//		q2q2 = q2 * q2;
//		q2q3 = q2 * q3;
//		q3q3 = q3 * q3;

//		// Reference direction of Earth's magnetic field
//		hx = hmc5883_unit.hx * q0q0 - _2q0my * q3 + _2q0mz * q2 + hmc5883_unit.hx * q1q1 + _2q1 * hmc5883_unit.hy * q2 + _2q1 * hmc5883_unit.hz * q3 - hmc5883_unit.hx * q2q2 - hmc5883_unit.hx * q3q3;
//		hy = _2q0mx * q3 + hmc5883_unit.hy * q0q0 - _2q0mz * q1 + _2q1mx * q2 - hmc5883_unit.hy * q1q1 + hmc5883_unit.hy * q2q2 + _2q2 * hmc5883_unit.hz * q3 - hmc5883_unit.hy * q3q3;
//		_2bx = sqrt(hx * hx + hy * hy);
//		_2bz = -_2q0mx * q2 + _2q0my * q1 + hmc5883_unit.hz * q0q0 + _2q1mx * q3 - hmc5883_unit.hz * q1q1 + _2q2 * hmc5883_unit.hy * q3 - hmc5883_unit.hz * q2q2 + hmc5883_unit.hz * q3q3;
//		_4bx = 2.0f * _2bx;
//		_4bz = 2.0f * _2bz;

//		// Gradient decent algorithm corrective step
//		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - mpu6050_unit1.ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - mpu6050_unit1.ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - hmc5883_unit.hx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - hmc5883_unit.hy) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - hmc5883_unit.hz);
//		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - mpu6050_unit1.ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - mpu6050_unit1.ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - mpu6050_unit1.az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - hmc5883_unit.hx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - hmc5883_unit.hy) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - hmc5883_unit.hz);
//		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - mpu6050_unit1.ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - mpu6050_unit1.ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - mpu6050_unit1.az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - hmc5883_unit.hx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - hmc5883_unit.hy) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - hmc5883_unit.hz);
//		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - mpu6050_unit1.ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - mpu6050_unit1.ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - hmc5883_unit.hx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - hmc5883_unit.hy) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - hmc5883_unit.hz);
//		recipNorm = InvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
//		s0 *= recipNorm;
//		s1 *= recipNorm;
//		s2 *= recipNorm;
//		s3 *= recipNorm;

//		// Apply feedback step
//		qDot1 -= beta * s0;
//		qDot2 -= beta * s1;
//		qDot3 -= beta * s2;
//		qDot4 -= beta * s3;
//		
//		
//	}

////	// Integrate rate of change of quaternion to yield quaternion
////	q0 += qDot1 * (1.0f / sampleFreq);
////	q1 += qDot2 * (1.0f / sampleFreq);
////	q2 += qDot3 * (1.0f / sampleFreq);
////	q3 += qDot4 * (1.0f / sampleFreq);
//		/* 将四元数姿态导数积分,得到当前四元数姿态 */
//	/* 二阶毕卡求解微分方程 */
//	delta = (T_cyc * mpu6050_unit1.gx) * (T_cyc * mpu6050_unit1.gx) + (T_cyc * mpu6050_unit1.gy) * (T_cyc * mpu6050_unit1.gy) +(T_cyc * mpu6050_unit1.gz) * (T_cyc * mpu6050_unit1.gz);
//	q0 = (1.0f - delta / 8.0f) * q0 + qDot1 * T_cyc;
//	q1 = (1.0f - delta / 8.0f) * q1 + qDot2 * T_cyc;
//	q2 = (1.0f - delta / 8.0f) * q2 + qDot3 * T_cyc;
//	q3 = (1.0f - delta / 8.0f) * q3 + qDot4 * T_cyc;

//	// Normalise quaternion
//	recipNorm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
//	q0 *= recipNorm;
//	q1 *= recipNorm;
//	q2 *= recipNorm;
//	q3 *= recipNorm;
//	
//			/*四元数方向余弦矩阵*/
//		t11=q0q0 + q1q1 - q2q2 - q3q3;
//		t12=2*q1q2 + 2*q0q3;
//		t13=2*q1q3 - 2*q0q2;
//		t23=2*q2q3 + 2*q0q1;
//		t33=q0q0 - q1q1 - q2q2 + q3q3;
//		/*导出姿态角 GYRO*/
//		angel_out->yaw   =  atan2(t12,t11) * Rad_To_Angle;
//		angel_out->pitch = -asin(t13) * Rad_To_Angle ;
//		angel_out->roll  =  atan2(t23,t33) * Rad_To_Angle;	
}

/***************************************************
函数名：Kalman_IMU_Update(MPU6050_DATA_Unit mpu6050_unit1,ANGEL *angel_out)
说明：卡尔曼滤波，姿态
入口：加速度计和陀螺仪国际单位数据，
出口：roll   pitch
备注：yaw是用磁强计得到
***************************************************/
float P_0[4][4] = {{ 1.0, 0, 0, 0}, {0, 1.0, 0, 0}, {0, 0, 1.0, 0}, {0, 0, 0, 1.0} };//协方差矩阵初值
float _I[4][4]  = {{ 1.0, 0, 0, 0}, {0, 1.0, 0, 0}, {0, 0, 1.0, 0}, {0, 0, 0, 1.0} };   //4X4单位矩阵
float Q_1[4] = { 1.0, 0.0, 0.0, 0.0 };  //当前四元数值
float Q_0[4];														//上一次四元数值
float Q_Fore_Noise = 0.001f;  
float R_Obser_Noise = 0.5f;
float Kg[4][4];
float Q_Obser[4];
float _Q_Obser[4];

void Kalman_IMU_Update(MPU6050_DATA_Unit mpu6050_unit1,ANGEL *angel_out)
{
//	u8 i,j;
//	float norm;
//	float _roll,_pitch;
//	float AT[4][4]; //A矩阵的转置
//	float P_AP[4][4],P_APAT[4][4],P_R[4][4],P_R_Inverce[4][4];
//	
//	float A[4][4]={{1.0, -mpu6050_unit1.gx*halfT, -mpu6050_unit1.gy*halfT, -mpu6050_unit1.gz*halfT}, 
//								{mpu6050_unit1.gx*halfT, 1.0,mpu6050_unit1.gz*halfT, -mpu6050_unit1.gy*halfT}, 
//								{mpu6050_unit1.gy*halfT, -mpu6050_unit1.gz*halfT, 1.0,mpu6050_unit1.gx*halfT},    //状态矩阵
//								{mpu6050_unit1.gz*halfT,mpu6050_unit1.gy*halfT,-mpu6050_unit1.gx*halfT, 1.0}};
//	for(i = 0; i < 4; i++)
//			Q_0[i]=Q_1[i];
//	/******第一步***********/
//	m_multiply(A[0],Q_0,Q_1, 4, 4, 4, 1);
//	/******第二步***********/
//	for(i = 0; i < 4; i++)	
//		for(j = 0; j < 4; j++)	
//				AT[j][i] = A[i][j];		    //A矩阵的转置                                   
//								
//	m_multiply(A[0],P_0[0],P_AP[0], 4, 4, 4, 4);
//	m_multiply(P_AP[0], AT[0],P_APAT[0], 4, 4, 4, 4);
//	for(i = 0; i < 4; i++)
//			P_APAT[i][i] += Q_Fore_Noise;
//	/******第三步***********/
//	for(i = 0; i < 4; i++)	
//		for(j = 0; j < 4; j++)	
//				P_R[i][j] = P_APAT[i][j];		 
//	for(i = 0; i < 4; i++)		
//				P_R[i][i] += R_Obser_Noise;
//	inverse_matrix(P_R[0],P_R_Inverce[0],4);
//	m_multiply(P_APAT[0],P_R_Inverce[0],Kg[0], 4, 4, 4, 4);		
//	/******第四步***********/							
//	_roll  = atan2(mpu6050_unit1.ay,mpu6050_unit1.az);
//	_pitch = -asin(mpu6050_unit1.ax/g_Eart);
//	Q_Obser[0]=cos(0/2.0f)*cos(_pitch/2.0f)*cos(_roll/2.0f)+sin(0/2.0f)*sin(_pitch/2.0f)*sin(_roll/2.0f);
//	Q_Obser[1]=cos(0/2.0f)*cos(_pitch/2.0f)*sin(_roll/2.0f)-sin(0/2.0f)*sin(_pitch/2.0f)*cos(_roll/2.0f);
//	Q_Obser[2]=cos(0/2.0f)*sin(_pitch/2.0f)*cos(_roll/2.0f)+sin(0/2.0f)*cos(_pitch/2.0f)*sin(_roll/2.0f);
//	Q_Obser[3]=sin(0/2.0f)*cos(_pitch/2.0f)*cos(_roll/2.0f)-cos(0/2.0f)*sin(_pitch/2.0f)*sin(_roll/2.0f);
//	
//	for (i = 0; i < 4; i++)
//		Q_Obser[i] -= Q_1[i];//
//	m_multiply(Kg[0],Q_Obser,Q_Obser,4, 4, 4, 1);
//	for (i = 0; i < 4; i++)
//		Q_1[i] += Q_Obser[i];
//	/******第五步***********/		
//	for(i = 0; i <4; i++)
//		for(j = 0; j <4; j++)
//			Kg[i][j] = _I[i][j]-Kg[i][j];
//	m_multiply(Kg[0], P_APAT[0], P_0[0], 4, 4, 4, 4);
//	
//	
//  /******结束*******/
//	norm = InvSqrt(Q_1[0] * Q_1[0] + Q_1[1] * Q_1[1] + Q_1[2] * Q_1[2] + Q_1[3] * Q_1[3]);
//	Q_1[0] *= norm;
//	Q_1[1] *= norm;
//	Q_1[2] *= norm;
//	Q_1[3] *= norm;
//	
//	angel_out->pitch = asin(-2 * Q_1[1] * Q_1[3] + 2 * Q_1[0]* Q_1[2])* 57.3 ; // pitch
//	angel_out->roll  = atan2(2 * Q_1[2] * Q_1[3] + 2 * Q_1[0] * Q_1[1], -2 * Q_1[1]* Q_1[1] - 2 * Q_1[2]* Q_1[2] + 1)* 57.3 ; 
	//angel_out->yaw   = atan2(2*(Q_1[1]*Q_1[2]+Q_1[0]*Q_1[3]),1-2*(Q_1[2]*Q_1[2]+Q_1[3]*Q_1[3]));
}

void Kalman_Grad_Update(MPU6050_DATA_Unit mpu6050_unit1,ANGEL *angel_out)
{
//	u8 i,j;
//	float norm;
//	float AT[4][4]; //A矩阵的转置
//	float _S[4];
//	float P_AP[4][4],P_APAT[4][4],P_R[4][4],P_R_Inverce[4][4];
//	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
//	
//	float A[4][4]={{1.0, -mpu6050_unit1.gx*halfT, -mpu6050_unit1.gy*halfT, -mpu6050_unit1.gz*halfT}, 
//								{mpu6050_unit1.gx*halfT, 1.0,mpu6050_unit1.gz*halfT, -mpu6050_unit1.gy*halfT}, 
//								{mpu6050_unit1.gy*halfT, -mpu6050_unit1.gz*halfT, 1.0,mpu6050_unit1.gx*halfT},    //状态矩阵
//								{mpu6050_unit1.gz*halfT,mpu6050_unit1.gy*halfT,-mpu6050_unit1.gx*halfT, 1.0}};
//	for(i = 0; i < 4; i++)
//			Q_0[i]=Q_1[i];
//	/******第一步***********/
//	m_multiply(A[0],Q_0,Q_1, 4, 4, 4, 1);
//	/******第二步***********/
//	for(i = 0; i < 4; i++)	
//		for(j = 0; j < 4; j++)	
//				AT[j][i] = A[i][j];		    //A矩阵的转置                                   
//								
//	m_multiply(A[0],P_0[0],P_AP[0], 4, 4, 4, 4);
//	m_multiply(P_AP[0], AT[0],P_APAT[0], 4, 4, 4, 4);
//	for(i = 0; i < 4; i++)
//			P_APAT[i][i] += Q_Fore_Noise;
//	/******第三步***********/
//	for(i = 0; i < 4; i++)	
//		for(j = 0; j < 4; j++)	
//				P_R[i][j] = P_APAT[i][j];		 
//	for(i = 0; i < 4; i++)		
//				P_R[i][i] += R_Obser_Noise;
//	inverse_matrix(P_R[0],P_R_Inverce[0],4);
//	m_multiply(P_APAT[0],P_R_Inverce[0],Kg[0], 4, 4, 4, 4);		
//	/******第四步***********/			
//	norm=InvSqrt(mpu6050_unit1.ax*mpu6050_unit1.ax + mpu6050_unit1.ay*mpu6050_unit1.ay + mpu6050_unit1.az*mpu6050_unit1.az);
//	mpu6050_unit1.ax *= norm;
//	mpu6050_unit1.ay *= norm;
//	mpu6050_unit1.az *= norm;	
//	
//		_2q0 = 2.0f * Q_0[0];
//		_2q1 = 2.0f * Q_0[1];
//		_2q2 = 2.0f * Q_0[2];
//		_2q3 = 2.0f * Q_0[3];
//		_4q0 = 4.0f * Q_0[0];
//		_4q1 = 4.0f * Q_0[1];
//		_4q2 = 4.0f * Q_0[2];
//		_8q1 = 8.0f * Q_0[1];
//		_8q2 = 8.0f * Q_0[2];
//		q0q0 = Q_0[0] * Q_0[0];
//		q1q1 = Q_0[1] * Q_0[1];
//		q2q2 = Q_0[2] * Q_0[2];
//		q3q3 = Q_0[3] * Q_0[3];

//		// Gradient decent algorithm corrective step
//		_S[0] = _4q0 * q2q2 + _2q2 * mpu6050_unit1.ax + _4q0 * q1q1 - _2q1 * mpu6050_unit1.ay;
//		_S[1] = _4q1 * q3q3 - _2q3 * mpu6050_unit1.ax + 4.0f * q0q0 * Q_0[1] - _2q0 * mpu6050_unit1.ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * mpu6050_unit1.az;
//		_S[2] = 4.0f * q0q0 * Q_0[2] + _2q0 * mpu6050_unit1.ax + _4q2 * q3q3 - _2q3 * mpu6050_unit1.ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * mpu6050_unit1.az;
//		_S[3] = 4.0f * q1q1 * Q_0[3] - _2q1 * mpu6050_unit1.ax + 4.0f * q2q2 * Q_0[3] - _2q2 * mpu6050_unit1.ay;
//		norm = InvSqrt(_S[0] * _S[0] + _S[1] * _S[1] + _S[2] * _S[2] + _S[3] * _S[3]); // normalise step magnitude
//		_S[0] *= norm;
//		_S[1] *= norm;
//		_S[2] *= norm;
//		_S[3] *= norm;
//	
//	for (i = 0; i < 4; i++)
//		Q_0[i] -= beta*_S[i];
//	for (i = 0; i < 4; i++)
//		Q_0[i] -= Q_1[i];//
//	m_multiply(Kg[0],Q_0,Q_Obser,4, 4, 4, 1);
//	for (i = 0; i < 4; i++)
//		Q_1[i] += Q_Obser[i];
//	/******第五步***********/		
//	for(i = 0; i <4; i++)
//		for(j = 0; j <4; j++)
//			Kg[i][j] = _I[i][j]-Kg[i][j];
//	m_multiply(Kg[0], P_APAT[0], P_0[0], 4, 4, 4, 4);
//	
//	
//  /******结束*******/
//	norm = InvSqrt(Q_1[0] * Q_1[0] + Q_1[1] * Q_1[1] + Q_1[2] * Q_1[2] + Q_1[3] * Q_1[3]);
//	Q_1[0] *= norm;
//	Q_1[1] *= norm;
//	Q_1[2] *= norm;
//	Q_1[3] *= norm;
//	
//	angel_out->pitch = asin(-2 * Q_1[1] * Q_1[3] + 2 * Q_1[0]* Q_1[2])* 57.3 ; // pitch
//	angel_out->roll  = atan2(2 * Q_1[2] * Q_1[3] + 2 * Q_1[0] * Q_1[1], -2 * Q_1[1]* Q_1[1] - 2 * Q_1[2]* Q_1[2] + 1)* 57.3 ; 
//	//angel_out->yaw   = atan2(2*(Q_1[1]*Q_1[2]+Q_1[0]*Q_1[3]),1-2*(Q_1[2]*Q_1[2]+Q_1[3]*Q_1[3]));
}

void Kalman_Grad_ALL_Update(MPU6050_DATA_Unit mpu6050_unit1,HMC5883L_DATA_Unit hmc5883_unit,ANGEL *angel_out)
{
//	u8 i,j;
//	float norm;
//	float AT[4][4]; //A矩阵的转置
//	float _S[4];
//	float P_AP[4][4],P_APAT[4][4],P_R[4][4],P_R_Inverce[4][4];
//	float _2q0mx,_2q0my,_2q0mz,_2q1mx,_2q0q2,_2q2q3;
//	float hx,hy,_2bx,_2bz,_4bx,_4bz;
//	float _2q0, _2q1, _2q2, _2q3 , q0q0, q1q1, q1q2, q2q2, q3q3, q0q1, q0q2, q0q3, q1q3, q2q3;
//	
//	float A[4][4]={{1.0, -mpu6050_unit1.gx*halfT, -mpu6050_unit1.gy*halfT, -mpu6050_unit1.gz*halfT}, 
//								{mpu6050_unit1.gx*halfT, 1.0,mpu6050_unit1.gz*halfT, -mpu6050_unit1.gy*halfT}, 
//								{mpu6050_unit1.gy*halfT, -mpu6050_unit1.gz*halfT, 1.0,mpu6050_unit1.gx*halfT},    //状态矩阵
//								{mpu6050_unit1.gz*halfT,mpu6050_unit1.gy*halfT,-mpu6050_unit1.gx*halfT, 1.0}};
//	for(i = 0; i < 4; i++)
//			Q_0[i]=Q_1[i];
//	/******第一步***********/
//	m_multiply(A[0],Q_0,Q_1, 4, 4, 4, 1);
//	/******第二步***********/
//	for(i = 0; i < 4; i++)	
//		for(j = 0; j < 4; j++)	
//				AT[j][i] = A[i][j];		    //A矩阵的转置                                   
//								
//	m_multiply(A[0],P_0[0],P_AP[0], 4, 4, 4, 4);
//	m_multiply(P_AP[0], AT[0],P_APAT[0], 4, 4, 4, 4);
//	for(i = 0; i < 4; i++)
//			P_APAT[i][i] += Q_Fore_Noise;
//	/******第三步***********/
//	for(i = 0; i < 4; i++)	
//		for(j = 0; j < 4; j++)	
//				P_R[i][j] = P_APAT[i][j];		 
//	for(i = 0; i < 4; i++)		
//				P_R[i][i] += R_Obser_Noise;
//	inverse_matrix(P_R[0],P_R_Inverce[0],4);
//	m_multiply(P_APAT[0],P_R_Inverce[0],Kg[0], 4, 4, 4, 4);		
//	/******第四步***********/		
//	norm=InvSqrt(mpu6050_unit1.ax*mpu6050_unit1.ax + mpu6050_unit1.ay*mpu6050_unit1.ay + mpu6050_unit1.az*mpu6050_unit1.az);
//	mpu6050_unit1.ax *= norm;
//	mpu6050_unit1.ay *= norm;
//	mpu6050_unit1.az *= norm;
//	
//	norm = InvSqrt(hmc5883_unit.hx * hmc5883_unit.hx + hmc5883_unit.hy * hmc5883_unit.hy + hmc5883_unit.hz * hmc5883_unit.hz);
//	hmc5883_unit.hx *= norm;
//	hmc5883_unit.hy *= norm;
//	hmc5883_unit.hz *= norm;

//		_2q0mx = 2.0f * Q_0[0] * hmc5883_unit.hx;
//		_2q0my = 2.0f * Q_0[0] * hmc5883_unit.hy;
//		_2q0mz = 2.0f * Q_0[0] * hmc5883_unit.hz;
//		_2q1mx = 2.0f * Q_0[1] * hmc5883_unit.hx;
//		_2q0 = 2.0f * Q_0[0];
//		_2q1 = 2.0f * Q_0[1];
//		_2q2 = 2.0f * Q_0[2];
//		_2q3 = 2.0f * Q_0[3];
//		_2q0q2 = 2.0f * Q_0[0] * Q_0[2];
//		_2q2q3 = 2.0f * Q_0[2] * Q_0[3];
//		q0q0 = Q_0[0] * Q_0[0];
//		q0q1 = Q_0[0] * Q_0[1];
//		q0q2 = Q_0[0] * Q_0[2];
//		q0q3 = Q_0[0] * Q_0[3];
//		q1q1 = Q_0[1] * Q_0[1];
//		q1q2 = Q_0[1] * Q_0[2];
//		q1q3 = Q_0[1] * Q_0[3];
//		q2q2 = Q_0[2] * Q_0[2];
//		q2q3 = Q_0[2] * Q_0[3];
//		q3q3 = Q_0[3] * Q_0[3];

//		// Reference direction of Earth's magnetic field
//		hx = hmc5883_unit.hx * q0q0 - _2q0my * Q_0[3] + _2q0mz * Q_0[2] + hmc5883_unit.hx * q1q1 + _2q1 * hmc5883_unit.hy * Q_0[2] + _2q1 * hmc5883_unit.hz * Q_0[3] - hmc5883_unit.hx * q2q2 - hmc5883_unit.hx * q3q3;
//		hy = _2q0mx * Q_0[3] + hmc5883_unit.hy * q0q0 - _2q0mz * Q_0[1] + _2q1mx * Q_0[2] - hmc5883_unit.hy * q1q1 + hmc5883_unit.hy * q2q2 + _2q2 * hmc5883_unit.hz * Q_0[3] - hmc5883_unit.hy * q3q3;
//		_2bx = sqrt(hx * hx + hy * hy);
//		_2bz = -_2q0mx * Q_0[2] + _2q0my * Q_0[1] + hmc5883_unit.hz * q0q0 + _2q1mx * Q_0[3] - hmc5883_unit.hz * q1q1 + _2q2 * hmc5883_unit.hy * Q_0[3] - hmc5883_unit.hz * q2q2 + hmc5883_unit.hz * q3q3;
//		_4bx = 2.0f * _2bx;
//		_4bz = 2.0f * _2bz;

//		// Gradient decent algorithm corrective step
//		_S[0] = -_2q2 * (2.0f * q1q3 - _2q0q2 - mpu6050_unit1.ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - mpu6050_unit1.ay) - _2bz * Q_0[2] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - hmc5883_unit.hx) + (-_2bx * Q_0[3] + _2bz * Q_0[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - hmc5883_unit.hy) + _2bx * Q_0[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - hmc5883_unit.hz);
//		_S[1] = _2q3 * (2.0f * q1q3 - _2q0q2 - mpu6050_unit1.ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - mpu6050_unit1.ay) - 4.0f * Q_0[1] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - mpu6050_unit1.az) + _2bz * Q_0[3] * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - hmc5883_unit.hx) + (_2bx * Q_0[2] + _2bz * Q_0[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - hmc5883_unit.hy) + (_2bx * Q_0[3] - _4bz * Q_0[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - hmc5883_unit.hz);
//		_S[2] = -_2q0 * (2.0f * q1q3 - _2q0q2 - mpu6050_unit1.ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - mpu6050_unit1.ay) - 4.0f * Q_0[2] * (1 - 2.0f * q1q1 - 2.0f * q2q2 - mpu6050_unit1.az) + (-_4bx * Q_0[2] - _2bz * Q_0[0]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - hmc5883_unit.hx) + (_2bx * Q_0[1] + _2bz * Q_0[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - hmc5883_unit.hy) + (_2bx * Q_0[0] - _4bz * Q_0[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - hmc5883_unit.hz);
//		_S[3] = _2q1 * (2.0f * q1q3 - _2q0q2 - mpu6050_unit1.ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - mpu6050_unit1.ay) + (-_4bx * Q_0[3] + _2bz * Q_0[1]) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - hmc5883_unit.hx) + (-_2bx * Q_0[0] + _2bz * Q_0[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - hmc5883_unit.hy) + _2bx * Q_0[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - hmc5883_unit.hz);

//		norm = InvSqrt(_S[0] * _S[0] + _S[1] * _S[1] + _S[2] * _S[2] + _S[3] * _S[3]); // normalise step magnitude
//		_S[0] *= norm;
//		_S[1] *= norm;
//		_S[2] *= norm;
//		_S[3] *= norm;
//	
//	for (i = 0; i < 4; i++)
//		Q_0[i] -= beta*_S[i];
//	for (i = 0; i < 4; i++)
//		Q_0[i] -= Q_1[i];//
//	m_multiply(Kg[0],Q_0,Q_Obser,4, 4, 4, 1);
//	for (i = 0; i < 4; i++)
//		Q_1[i] += Q_Obser[i];
//	/******第五步***********/		
//	for(i = 0; i <4; i++)
//		for(j = 0; j <4; j++)
//			Kg[i][j] = _I[i][j]-Kg[i][j];
//	m_multiply(Kg[0], P_APAT[0], P_0[0], 4, 4, 4, 4);
//	
//	
//  /******结束*******/
//	norm = InvSqrt(Q_1[0] * Q_1[0] + Q_1[1] * Q_1[1] + Q_1[2] * Q_1[2] + Q_1[3] * Q_1[3]);
//	Q_1[0] *= norm;
//	Q_1[1] *= norm;
//	Q_1[2] *= norm;
//	Q_1[3] *= norm;
//	
//	angel_out->pitch = asin(-2 * Q_1[1] * Q_1[3] + 2 * Q_1[0]* Q_1[2])* 57.3 ; // pitch
//	angel_out->roll  = atan2(2 * Q_1[2] * Q_1[3] + 2 * Q_1[0] * Q_1[1], -2 * Q_1[1]* Q_1[1] - 2 * Q_1[2]* Q_1[2] + 1)* 57.3 ; 
//	angel_out->yaw   = atan2(2*(Q_1[1]*Q_1[2]+Q_1[0]*Q_1[3]),1-2*(Q_1[2]*Q_1[2]+Q_1[3]*Q_1[3])) * 57.3;
}

/*************************************************

**************************************************/
float InvSqrt(float x)
{
	float xhalf = 0.5f * x;
	int i = *(int*)&x; // get bits for floating value
	i = 0x5f375a86 - (i>>1); // gives initial guess
	x = *(float*)&i; // convert bits back to float
	x = x * (1.5f - xhalf*x*x); // Newton step
	return x;
}

/*************************************************
输入：遥控数据
输出：解锁标志
**************************************************/
void Rc_Fun(T_RC_Data *rc_in,uint8_t *armed)
{
	static uint16_t cnt_arm=0,i = 0; //,cnt_race=0,j=0
	/*油门和航向角都最小的时候，飞控解锁*/
	if(!SYSTEM_STA.ALT_HOLD_MODE && rc_in->THROTTLE < RC_FUN_MIN && rc_in->YAW < RC_FUN_MIN&& rc_in->PITCH > RC_FUN_MIN && rc_in->ROLL > RC_FUN_MIN)
	{
		/*******遥控器打开********/
		cnt_arm++;
		if(cnt_arm == 500)
		{
			cnt_arm=0;
			*armed = 1;
			blue_On();				
		}				
	}
	/****油门小，航向角大的时候，飞控上锁*****/
	else if(rc_in->THROTTLE < RC_FUN_MIN && rc_in->YAW > RC_FUN_MAX && rc_in->PITCH > RC_FUN_MIN && rc_in->ROLL > RC_FUN_MIN)
	{
		cnt_arm++;
		if(cnt_arm==500)
		{
			cnt_arm=0;
			*armed = 0;
			blue_Off();
		}
	}
	/****油门小，航向角大的时候，飞控上锁*****/
	else if(rc_in->THROTTLE < RC_FUN_MIN && rc_in->YAW > RC_FUN_MIN && rc_in->PITCH > RC_FUN_MIN && rc_in->ROLL > RC_FUN_MIN)
	{
		cnt_arm++;
		if(cnt_arm==500)
		{
			cnt_arm=0;
			*armed = 0;
			blue_Off();
		}
	}
	else
		cnt_arm = 0;
	
	/*俯仰和横滚角都最小的时候，飞控内环控制*/
//	if(rc_in->THROTTLE < RC_FUN_MIN && rc_in->YAW > RC_FUN_MIN&& rc_in->PITCH < RC_FUN_MIN && rc_in->ROLL < RC_FUN_MIN)
//	{
//		/*******内环打开********/
//		cnt_race++;
//		if(cnt_race==500)
//		{
//			cnt_race=0;
//			Inner_Outer_flag=0;	
//			for(j=0;j<3;j++)
//				blue_twinkle();
//		}
//			
//	}
//	/****fuyang小，滚转角大的时候，飞控外环控制*****/
//	else if(rc_in->THROTTLE < RC_FUN_MIN && rc_in->YAW > RC_FUN_MIN && rc_in->PITCH < RC_FUN_MIN && rc_in->ROLL > RC_FUN_MAX)
//	{
//		cnt_race++;
//		if(cnt_race==500)
//		{
//			cnt_race=0;
//			Inner_Outer_flag=1;	
//			for(j=0;j<3;j++)
//				blue_twinkle();
//		}
//	}
//	else 
//		cnt_race=0;
	/*************遥控器没有打开******************/
	if(rc_in->THROTTLE <1000)
	{
		i++;
		*armed = 0;	//遥控器没打开，所有通道输出最小值，飞控不解锁		
		if(i == 50)
		{
			i = 0;
			LED2 = !LED2;
		}			
	/****飞机失控保护代码添加******/			
	}
	/*****************************/	
	if(rc_in->THROTTLE >999 && *armed == 0)
		blue_Off();		
	if(*armed == 1) 
		return;
}


//float P_0[4][4] = {{ 1.0, 0, 0, 0}, {0, 1.0, 0, 0}, {0, 0, 1.0, 0}, {0, 0, 0, 1.0} };//D-�?2????�3??�
//float _I[4][4]  = {{ 1.0, 0, 0, 0}, {0, 1.0, 0, 0}, {0, 0, 1.0, 0}, {0, 0, 0, 1.0} };   //4X4�??????�
//float Q_1[4] = { 1.0, 0.0, 0.0, 0.0 };  //当?�???a陏?�
//float Q_0[4];														//�?�?'????a陏?�
//float Q_Fore_Noise = 0.001f;  
//float R_Obser_Noise = 0.5f;
//float Kg[4][4];
//float Q_Obser[4];

//void Kalman_IMU_Update(MPU6050_DATA_Unit mpu6050_unit1,ANGEL *angel_out)
//{
//	u8 i,j;
//	float norm;
//	float _roll,_pitch;
//	float AT[4][4]; //A???蟮?譨??
//	float P_AP[4][4],P_APAT[4][4],P_R[4][4],P_R_Inverce[4][4];
//	
//	
//	float A[4][4]={{1.0, -mpu6050_unit1.gx*halfT, -mpu6050_unit1.gy*halfT, -mpu6050_unit1.gz*halfT}, 
//								{mpu6050_unit1.gx*halfT, 1.0,mpu6050_unit1.gz*halfT, -mpu6050_unit1.gy*halfT}, 
//								{mpu6050_unit1.gy*halfT, -mpu6050_unit1.gz*halfT, 1.0,mpu6050_unit1.gx*halfT},    //�'�????�
//								{mpu6050_unit1.gz*halfT,mpu6050_unit1.gy*halfT,-mpu6050_unit1.gx*halfT, 1.0}};
//	for(i = 0; i < 4; i++)
//			Q_0[i]=Q_1[i];
//	/******碟�?2?***********/
//	m_multiply(A[0],Q_0,Q_1, 4, 4, 4, 1);
//	/******碟?t2?***********/
//	for(i = 0; i < 4; i++)	
//		for(j = 0; j < 4; j++)	
//				AT[j][i] = A[i][j];		    //A???蟮?譨??                                   
//								
//	m_multiply(A[0],P_0[0],P_AP[0], 4, 4, 4, 4);
//	m_multiply(P_AP[0], AT[0],P_APAT[0], 4, 4, 4, 4);
//	for(i = 0; i < 4; i++)
//			P_APAT[i][i] += Q_Fore_Noise;
//	/******碟鑩2?***********/
//	for(i = 0; i < 4; i++)	
//		for(j = 0; j < 4; j++)	
//				P_R[i][j] = P_APAT[i][j];		    //A???蟮?譨??   
//	for(i = 0; i < 4; i++)		
//				P_R[i][i] += R_Obser_Noise;
//	inverse_matrix(P_R[0],P_R_Inverce[0],4);
//	m_multiply(P_APAT[0],P_R_Inverce[0],Kg[0], 4, 4, 4, 4);		
//	/******碟??2?***********/							
//	_roll  = atan2(mpu6050_unit1.ay,mpu6050_unit1.az);
//	_pitch = -asin(mpu6050_unit1.ax/g_Eart);
//	Q_Obser[0]=cos(0/2.0f)*cos(_pitch/2.0f)*cos(_roll/2.0f)+sin(0/2.0f)*sin(_pitch/2.0f)*sin(_roll/2.0f);
//	Q_Obser[1]=cos(0/2.0f)*cos(_pitch/2.0f)*sin(_roll/2.0f)-sin(0/2.0f)*sin(_pitch/2.0f)*cos(_roll/2.0f);
//	Q_Obser[2]=cos(0/2.0f)*sin(_pitch/2.0f)*cos(_roll/2.0f)+sin(0/2.0f)*cos(_pitch/2.0f)*sin(_roll/2.0f);
//	Q_Obser[3]=sin(0/2.0f)*cos(_pitch/2.0f)*cos(_roll/2.0f)-cos(0/2.0f)*sin(_pitch/2.0f)*sin(_roll/2.0f);
//	
//	for (i = 0; i < 4; i++)
//		Q_Obser[i] -= Q_1[i];//
//	m_multiply(Kg[0],Q_Obser,Q_Obser,4, 4, 4, 1);
//	for (i = 0; i < 4; i++)
//		Q_1[i] += Q_Obser[i];
//	/******碟??2?***********/		
//	for(i = 0; i <4; i++)
//		for(j = 0; j <4; j++)
//			Kg[i][j] = _I[i][j]-Kg[i][j];
//	m_multiply(Kg[0], P_APAT[0], P_0[0], 4, 4, 4, 4);
//	
//	
//  /******?彡?*******/
//	norm = InvSqrt(Q_1[0] * Q_1[0] + Q_1[1] * Q_1[1] + Q_1[2] * Q_1[2] + Q_1[3] * Q_1[3]);
//	Q_1[0] *= norm;
//	Q_1[1] *= norm;
//	Q_1[2] *= norm;
//	Q_1[3] *= norm;
//	
//	angel_out->pitch = asin(-2 * Q_1[1] * Q_1[3] + 2 * Q_1[0]* Q_1[2])* 57.3 ; // pitch
//	angel_out->roll  = atan2(2 * Q_1[2] * Q_1[3] + 2 * Q_1[0] * Q_1[1], -2 * Q_1[1]* Q_1[1] - 2 * Q_1[2]* Q_1[2] + 1)* 57.3 ; 
//	//angel_out->yaw   = atan2(2*(Q_1[1]*Q_1[2]+Q_1[0]*Q_1[3]),1-2*(Q_1[2]*Q_1[2]+Q_1[3]*Q_1[3]));
//}

