#include "flash.h"
#define EE_6050_ACC_X_OFFSET_ADDR	0
#define EE_6050_ACC_Y_OFFSET_ADDR	1
#define EE_6050_ACC_Z_OFFSET_ADDR	2
#define EE_6050_GYRO_X_OFFSET_ADDR	3
#define EE_6050_GYRO_Y_OFFSET_ADDR	4
#define EE_6050_GYRO_Z_OFFSET_ADDR	5
#define EE_PID_ROL_P	6
#define EE_PID_ROL_I	7
#define EE_PID_ROL_D	8
#define EE_PID_PIT_P	9
#define EE_PID_PIT_I	10
#define EE_PID_PIT_D	11
#define EE_PID_YAW_P	12
#define EE_PID_YAW_I	13
#define EE_PID_YAW_D	14
#define EE_PID_ALT_P	15
#define EE_PID_ALT_I	16
#define EE_PID_ALT_D	17
#define EE_PID_POS_P	18
#define EE_PID_POS_I	19
#define EE_PID_POS_D	20

#define EE_CMP_X_OFFSET_ADDR	21
#define EE_CMP_Y_OFFSET_ADDR	22
#define EE_CMP_Z_OFFSET_ADDR	23
#define EE_BARO_OFFSET_ADDRL	24

#define EE_CMP_Y_GAIN_ADDR	25
#define EE_CMP_Z_GAIN_ADDR	26

#define EE_PID_PID1_P	27
#define EE_PID_PID1_I	28
#define EE_PID_PID1_D	29

#define EE_PID_ROLL_RATE_P	30
#define EE_PID_ROLL_RATE_I	31
#define EE_PID_ROLL_RATE_D	32

#define EE_PID_PITCH_RATE_P	33
#define EE_PID_PITCH_RATE_I	34
#define EE_PID_PITCH_RATE_D	35

#define EE_PID_YAW_RATE_P	36
#define EE_PID_YAW_RATE_I	37
#define EE_PID_YAW_RATE_D	38

#define EE_PID_ALT_RATE_P   39
#define EE_PID_ALT_RATE_I   40
#define EE_PID_ALT_RATE_D   41

#define EE_PID_POS_RATE_P   42
#define EE_PID_POS_RATE_I   43
#define EE_PID_POS_RATE_D   44

#define EE_BARO_OFFSET_ADDRH 45

#define EE_PID1_RATE_P   46
#define EE_PID1_RATE_I   47
#define EE_PID1_RATE_D   48


//ErrorStatus  HSEStartUpStatus;
//FLASH_Status FlashStatus;
uint16_t VirtAddVarTab[NumbOfVar] = {0xAA00, 0xAA01, 0xAA02, 0xAA03, 0xAA04, 0xAA05, 0xAA06, 0xAA07, 0xAA08, 0xAA09, 0xAA0A, 0xAA0B, 0xAA0C, 0xAA0D, 0xAA0E, 0xAA0F,
									 0xAA10, 0xAA11, 0xAA12, 0xAA13, 0xAA14, 0xAA15, 0xAA16, 0xAA17, 0xAA18,0xAA19, 0xAA1A,
									 0xAA1B,0xAA1C,0xAA1D,0xAA1E,0xAA1F,0xAA20,
									 0xAA21,0xAA22,0xAA23,0xAA24,0xAA25,0xAA26,
									 0xAA27,0xAA28,0xAA29,0xAA2A,0xAA2B,0xAA2C,
									 0xAA2D,0xAA2E,0xAA2F,0xAA30};
uint16_t temp;
void EE_INIT(void)
{
	EE_Init();
}
void EE_SAVE_ACC_OFFSET(void)
{
	EE_WriteVariable(VirtAddVarTab[EE_6050_ACC_X_OFFSET_ADDR], MPU6050_OFFSER.ax);
	EE_WriteVariable(VirtAddVarTab[EE_6050_ACC_Y_OFFSET_ADDR], MPU6050_OFFSER.ay);
	EE_WriteVariable(VirtAddVarTab[EE_6050_ACC_Z_OFFSET_ADDR], MPU6050_OFFSER.az);
}
void EE_READ_ACC_OFFSET(void)
{
	uint16_t _temp; 
	EE_ReadVariable(VirtAddVarTab[EE_6050_ACC_X_OFFSET_ADDR], &_temp);
	MPU6050_OFFSER.ax = _temp;
	EE_ReadVariable(VirtAddVarTab[EE_6050_ACC_Y_OFFSET_ADDR], &_temp);
	MPU6050_OFFSER.ay = _temp;
	EE_ReadVariable(VirtAddVarTab[EE_6050_ACC_Z_OFFSET_ADDR], &_temp);
	MPU6050_OFFSER.az = _temp;	
}
void EE_SAVE_GYRO_OFFSET(void)
{
	EE_WriteVariable(VirtAddVarTab[EE_6050_GYRO_X_OFFSET_ADDR], MPU6050_OFFSER.gx);
	EE_WriteVariable(VirtAddVarTab[EE_6050_GYRO_Y_OFFSET_ADDR], MPU6050_OFFSER.gy);
	EE_WriteVariable(VirtAddVarTab[EE_6050_GYRO_Z_OFFSET_ADDR], MPU6050_OFFSER.gz);
}
void EE_READ_GYRO_OFFSET(void)
{
	uint16_t _temp; 
	EE_ReadVariable(VirtAddVarTab[EE_6050_GYRO_X_OFFSET_ADDR], &_temp);
	MPU6050_OFFSER.gx = _temp;
	EE_ReadVariable(VirtAddVarTab[EE_6050_GYRO_Y_OFFSET_ADDR], &_temp);
	MPU6050_OFFSER.gy = _temp;
	EE_ReadVariable(VirtAddVarTab[EE_6050_GYRO_Z_OFFSET_ADDR], &_temp);
	MPU6050_OFFSER.gz = _temp;
}

void EE_SAVE_CMP_OFFSET(void)
{
	uint16_t _temp;
	_temp = (uint16_t)hmc_offset.hx;
	EE_WriteVariable(VirtAddVarTab[EE_CMP_X_OFFSET_ADDR], _temp);
	_temp = (uint16_t)hmc_offset.hy;
	EE_WriteVariable(VirtAddVarTab[EE_CMP_Y_OFFSET_ADDR], _temp);
	_temp = (uint16_t)hmc_offset.hz;
	EE_WriteVariable(VirtAddVarTab[EE_CMP_Z_OFFSET_ADDR], _temp);
	
	_temp = (uint16_t)(My_Gain * 10000);
	EE_WriteVariable(VirtAddVarTab[EE_CMP_Y_GAIN_ADDR], _temp);
	_temp = (uint16_t)(Mz_Gain * 10000);
	EE_WriteVariable(VirtAddVarTab[EE_CMP_Z_GAIN_ADDR], _temp);	
}

void EE_READ_CMP_OFFSET(void)
{
	uint16_t _temp; 
	EE_ReadVariable(VirtAddVarTab[EE_CMP_X_OFFSET_ADDR], &_temp);
	hmc_offset.hx = (int16_t)_temp;
	EE_ReadVariable(VirtAddVarTab[EE_CMP_Y_OFFSET_ADDR], &_temp);
	hmc_offset.hy = (int16_t)_temp;
	EE_ReadVariable(VirtAddVarTab[EE_CMP_Z_OFFSET_ADDR], &_temp);	
	hmc_offset.hz = (int16_t)_temp;			
	EE_ReadVariable(VirtAddVarTab[EE_CMP_Y_GAIN_ADDR], &_temp);
	My_Gain = (float)( _temp/10000.0f);
	EE_ReadVariable(VirtAddVarTab[EE_CMP_Z_GAIN_ADDR], &_temp);	
	Mz_Gain = (float)( _temp/10000.0f);		
}

void EE_READ_BARO_OFFSET(void)
{ 
	uint16_t _temp1,_temp2;
	EE_ReadVariable(VirtAddVarTab[EE_BARO_OFFSET_ADDRL], &_temp1);
	EE_ReadVariable(VirtAddVarTab[EE_BARO_OFFSET_ADDRH], &_temp2);	
	BaroOffset = (vs32)(_temp2<<16 | _temp1);
}

void EE_SAVE_BARO_OFFSET(void)
{	
	uint16_t _temp1,_temp2;
	_temp1 = D_BYTE0(BaroOffset);
	_temp2 = D_BYTE1(BaroOffset);
	EE_WriteVariable(VirtAddVarTab[EE_BARO_OFFSET_ADDRL],_temp1);
	EE_WriteVariable(VirtAddVarTab[EE_BARO_OFFSET_ADDRH],_temp2);	
}

void EE_SAVE_PID1(void)
{
	uint16_t _temp;
	_temp = (uint16_t)(PID1_ipt.ROL_P * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_ROL_P],_temp);
	_temp = (uint16_t)(PID1_ipt.ROL_I * 1000);
	EE_WriteVariable(VirtAddVarTab[EE_PID_ROL_I],_temp);
	_temp = (uint16_t)(PID1_ipt.ROL_D * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_ROL_D],_temp);
	
	_temp = (uint16_t)(PID1_ipt.PIT_P * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_PIT_P],_temp);
	_temp = (uint16_t)(PID1_ipt.PIT_I * 1000);
	EE_WriteVariable(VirtAddVarTab[EE_PID_PIT_I],_temp);
	_temp = (uint16_t)(PID1_ipt.PIT_D * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_PIT_D],_temp);
	
	_temp = (uint16_t)(PID1_ipt.YAW_P * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_YAW_P],_temp);
	_temp = (uint16_t)(PID1_ipt.YAW_I * 1000);
	EE_WriteVariable(VirtAddVarTab[EE_PID_YAW_I],_temp);
	_temp = (uint16_t)(PID1_ipt.YAW_D * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_YAW_D],_temp);
}

void EE_SAVE_PID2(void)	
{
	uint16_t _temp;
	_temp = (uint16_t)(PID1_ipt.ALT_P * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_ALT_P],_temp);
	_temp = (uint16_t)(PID1_ipt.ALT_I * 1000);
	EE_WriteVariable(VirtAddVarTab[EE_PID_ALT_I],_temp);
	_temp = (uint16_t)(PID1_ipt.ALT_D * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_ALT_D],_temp);
	
	_temp = (uint16_t)(PID1_ipt.POS_P * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_POS_P],_temp);
	_temp = (uint16_t)(PID1_ipt.POS_I * 1000);
	EE_WriteVariable(VirtAddVarTab[EE_PID_POS_I],_temp);
	_temp = (uint16_t)(PID1_ipt.POS_D * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_POS_D],_temp);
	
	_temp = (uint16_t)(PID1_ipt.PID1_P * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_PID1_P],_temp);
	_temp = (uint16_t)(PID1_ipt.PID1_I * 10000);//数值很小，缩放很大
	EE_WriteVariable(VirtAddVarTab[EE_PID_PID1_I],_temp);
	_temp = (uint16_t)(PID1_ipt.PID1_D * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_PID1_D],_temp);	
}

void EE_SAVE_PID3(void)
{
	uint16_t _temp;
	_temp = (uint16_t)(PID2_ipt.ROL_RATE_P * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_ROLL_RATE_P],_temp);
	_temp = (uint16_t)(PID2_ipt.ROL_RATE_I * 1000);
	EE_WriteVariable(VirtAddVarTab[EE_PID_ROLL_RATE_I],_temp);
	_temp = (uint16_t)(PID2_ipt.ROL_RATE_D * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_ROLL_RATE_D],_temp);
	
	_temp = (uint16_t)(PID2_ipt.PIT_RATE_P * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_PITCH_RATE_P],_temp);
	_temp = (uint16_t)(PID2_ipt.PIT_RATE_I * 1000);
	EE_WriteVariable(VirtAddVarTab[EE_PID_PITCH_RATE_I],_temp);
	_temp = (uint16_t)(PID2_ipt.PIT_RATE_D * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_PITCH_RATE_D],_temp);
	
	_temp = (uint16_t)(PID2_ipt.YAW_RATE_P * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_YAW_RATE_P],_temp);
	_temp = (uint16_t)(PID2_ipt.YAW_RATE_I * 1000);
	EE_WriteVariable(VirtAddVarTab[EE_PID_YAW_RATE_I],_temp);
	_temp = (uint16_t)(PID2_ipt.YAW_RATE_D * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_YAW_RATE_D],_temp);	
}

void EE_SAVE_PID4(void)
{
	uint16_t _temp;
	_temp = (uint16_t)(PID2_ipt.ALT_RATE_P * 100);   //
	EE_WriteVariable(VirtAddVarTab[EE_PID_ALT_RATE_P],_temp);
	_temp = (uint16_t)(PID2_ipt.ALT_RATE_I * 1000);
	EE_WriteVariable(VirtAddVarTab[EE_PID_ALT_RATE_I],_temp);
	_temp = (uint16_t)(PID2_ipt.ALT_RATE_D * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_ALT_RATE_D],_temp);
	
	_temp = (uint16_t)(PID2_ipt.POS_RATE_P * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_POS_RATE_P],_temp);
	_temp = (uint16_t)(PID2_ipt.POS_RATE_I * 1000);
	EE_WriteVariable(VirtAddVarTab[EE_PID_POS_RATE_I],_temp);
	_temp = (uint16_t)(PID2_ipt.POS_RATE_D * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID_POS_RATE_D],_temp);		
	
	_temp = (uint16_t)(PID2_ipt.PID1_RATE_P * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID1_RATE_P],_temp);
	_temp = (uint16_t)(PID2_ipt.PID1_RATE_I * 1000);
	EE_WriteVariable(VirtAddVarTab[EE_PID1_RATE_I],_temp);
	_temp = (uint16_t)(PID2_ipt.PID1_RATE_D * 100);
	EE_WriteVariable(VirtAddVarTab[EE_PID1_RATE_D],_temp);		
}

void EE_READ_PID(void)
{
	uint16_t _temp;
	EE_ReadVariable(VirtAddVarTab[EE_PID_ROL_P],&_temp);
	PID1_ipt.ROL_P = (float)(_temp / 100.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_ROL_I],&_temp);
	PID1_ipt.ROL_I = (float)(_temp / 1000.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_ROL_D],&_temp);
	PID1_ipt.ROL_D = (float)(_temp / 100.0f);
	
	EE_ReadVariable(VirtAddVarTab[EE_PID_PIT_P],&_temp);
	PID1_ipt.PIT_P = (float)(_temp / 100.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_PIT_I],&_temp);
	PID1_ipt.PIT_I = (float)(_temp / 1000.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_PIT_D],&_temp);
	PID1_ipt.PIT_D = (float)(_temp / 100.0f);
		
	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_P],&_temp);
	PID1_ipt.YAW_P = (float)(_temp / 100.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_I],&_temp);
	PID1_ipt.YAW_I = (float)(_temp / 1000.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_D],&_temp);
	PID1_ipt.YAW_D = (float)(_temp / 100.0f);
	
	EE_ReadVariable(VirtAddVarTab[EE_PID_ALT_P],&_temp);
	PID1_ipt.ALT_P = (float)(_temp / 100.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_ALT_I],&_temp);
	PID1_ipt.ALT_I = (float)(_temp / 1000.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_ALT_D],&_temp);
	PID1_ipt.ALT_D = (float)(_temp / 100.0f);
	
	EE_ReadVariable(VirtAddVarTab[EE_PID_POS_P],&_temp);
	PID1_ipt.POS_P = (float)(_temp / 100.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_POS_I],&_temp);
	PID1_ipt.POS_I = (float)(_temp / 1000.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_POS_D],&_temp);
	PID1_ipt.POS_D = (float)(_temp / 100.0f);

	EE_ReadVariable(VirtAddVarTab[EE_PID_PID1_P],&_temp);
	PID1_ipt.PID1_P = (float)(_temp / 100.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_PID1_I],&_temp);
	PID1_ipt.PID1_I = (float)(_temp / 10000.0f);//数值很小，缩放很大
	EE_ReadVariable(VirtAddVarTab[EE_PID_PID1_D],&_temp);
	PID1_ipt.PID1_D = (float)(_temp / 100.0f);

	EE_ReadVariable(VirtAddVarTab[EE_PID_ROLL_RATE_P],&_temp);
	PID2_ipt.ROL_RATE_P = (float)(_temp / 100.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_ROLL_RATE_I],&_temp);
	PID2_ipt.ROL_RATE_I = (float)(_temp / 1000.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_ROLL_RATE_D],&_temp);
	PID2_ipt.ROL_RATE_D = (float)(_temp / 100.0f);
	
	EE_ReadVariable(VirtAddVarTab[EE_PID_PITCH_RATE_P],&_temp);
	PID2_ipt.PIT_RATE_P = (float)(_temp / 100.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_PITCH_RATE_I],&_temp);
	PID2_ipt.PIT_RATE_I = (float)(_temp / 1000.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_PITCH_RATE_D],&_temp);
	PID2_ipt.PIT_RATE_D = (float)(_temp / 100.0f);
		
	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_RATE_P],&_temp);
	PID2_ipt.YAW_RATE_P = (float)(_temp / 100.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_RATE_I],&_temp);
	PID2_ipt.YAW_RATE_I = (float)(_temp / 1000.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_RATE_D],&_temp);
	PID2_ipt.YAW_RATE_D = (float)(_temp / 100.0f);
	
	EE_ReadVariable(VirtAddVarTab[EE_PID_ALT_RATE_P],&_temp);
	PID2_ipt.ALT_RATE_P = (float)(_temp / 100.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_ALT_RATE_I],&_temp);
	PID2_ipt.ALT_RATE_I = (float)(_temp / 1000.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_ALT_RATE_D],&_temp);
	PID2_ipt.ALT_RATE_D = (float)(_temp / 100.0f);
	
	EE_ReadVariable(VirtAddVarTab[EE_PID_POS_RATE_P],&_temp);
	PID2_ipt.POS_RATE_P = (float)(_temp / 100.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_POS_RATE_I],&_temp);
	PID2_ipt.POS_RATE_I = (float)(_temp / 1000.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID_POS_RATE_D],&_temp);
	PID2_ipt.POS_RATE_D = (float)(_temp / 100.0f);
	
	EE_ReadVariable(VirtAddVarTab[EE_PID1_RATE_P],&_temp);
	PID2_ipt.PID1_RATE_P = (float)(_temp / 100.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID1_RATE_I],&_temp);
	PID2_ipt.PID1_RATE_I = (float)(_temp / 1000.0f);
	EE_ReadVariable(VirtAddVarTab[EE_PID1_RATE_D],&_temp);
	PID2_ipt.PID1_RATE_D = (float)(_temp / 100.0f);
}



