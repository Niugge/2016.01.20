#include "HMC5883L.h"

#define HEAD_LPF_FACTOR 20

/******************************************************************************
/ 函数功能:初始化HMC5883
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:none
******************************************************************************/
void  HMC5883L_Init(void)
{
   Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x14);   //30Hz
   Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x00);   //连续测量模式
}

/******************************************************************************
/ 函数功能:读取HMC5883的数据，单位国际化
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:none
******************************************************************************/
void HMC5883L_Read(HMC5883L_DATA * ptResult,HMC5883L_DATA_Unit * hmc5883_calibration)
{
    static uint8_t tmp[6];
	float mx_sum = 0, my_sum = 0, mz_sum = 0;
	/**************************/
    tmp[0]=Single_Read(HMC5883L_Addr,HMC5883L_HX_H);//OUT_X_L_A
    tmp[1]=Single_Read(HMC5883L_Addr,HMC5883L_HX_L);//OUT_X_H_A
    
    tmp[2]=Single_Read(HMC5883L_Addr,HMC5883L_HZ_H);//OUT_Z_L_A
    tmp[3]=Single_Read(HMC5883L_Addr,HMC5883L_HZ_L);//OUT_Z_H_A
    
    tmp[4]=Single_Read(HMC5883L_Addr,HMC5883L_HY_H);//OUT_Y_L_A
    tmp[5]=Single_Read(HMC5883L_Addr,HMC5883L_HY_L);//OUT_Y_H_A

	ptResult->hx    = (int16_t)((tmp[0] << 8) | tmp[1]);
	ptResult->hy    = (int16_t)((tmp[4] << 8) | tmp[5]);
    ptResult->hz    = (int16_t)((tmp[2] << 8) | tmp[3]);
	/****************归0归1***************/	
	if(CMP_OFFSET_OK)
	{
		if(cal_cnt)
		{
			cal_cnt--;
			if(mx_min > ptResult->hx)	mx_min = ptResult->hx;
			if(mx_max < ptResult->hx)	mx_max = ptResult->hx;
			if(my_min > ptResult->hy)	my_min = ptResult->hy;
			if(my_max < ptResult->hy)	my_max = ptResult->hy;
			if(mz_min > ptResult->hz)	mz_min = ptResult->hz;
			if(mz_max < ptResult->hz)	mz_max = ptResult->hz;
			if(cal_cnt==0)
			{
				static uint8_t j = 0;
				hmc_offset.hx = (mx_min + mx_max)/2;
				hmc_offset.hy = (my_min + my_max)/2;
				hmc_offset.hz = (mz_min + mz_max)/2;
				
				mx_sum = mx_max - mx_min;
				my_sum = my_max - my_min;
				mz_sum = mz_max - mz_min;
				
				My_Gain = mx_sum/my_sum;
				Mz_Gain = mx_sum/mz_sum;
				CMP_OFFSET_OK = 0;
				EE_SAVE_CMP_OFFSET();
			    for(j=0;j<3;j++)
					{all_ON(); delay_ms(300);all_OFF();delay_ms(300);}
			}		
		}		
	}
	
	else	
	{
		hmc5883_calibration->hx = (float)(ptResult->hx  - hmc_offset.hx);
		hmc5883_calibration->hy = (float)((ptResult->hy - hmc_offset.hy) * My_Gain);
		hmc5883_calibration->hz = (float)((ptResult->hz - hmc_offset.hz) * Mz_Gain);
	}
}
/******************************************************************************
/ 函数功能:HMC5883校准
/ 修改日期:none
/ 输入参数:none
/ 输出参数:none
/ 使用说明:启动-中断-查询 (查询)
******************************************************************************/
void HMC5883L_Calibrate(void)
{
   Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x15);   //30Hz,启动自检模式
   Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x01);   //单一测量模式
   delay_ms(10);
   Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x14);
   Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x00);   //回到工作模式
}

/******************************************************************************
/ 函数功能:单字节写入
/ 修改日期:none
/ 输入参数:
/   @arg SlaveAddress   从器件地址
/   @arg REG_Address    寄存器地址
/ 输出参数: 读出的字节数据
/ 使用说明:这时一个完整的单字节读取函数
******************************************************************************/
uint8_t Single_Read(uint8_t SlaveAddress,uint8_t REG_Address)
{   
    uint8_t REG_data;       
    I2C_Start();//if(!I2C_Start())return 0
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    I2C_WaitAck();//if(!I2C_WaitAck()){I2C_Stop();return 0;}
    I2C_SendByte((uint8_t) REG_Address);   //设置低起始地址      
    I2C_WaitAck();//if(!I2C_WaitAck()){I2C_Stop();return 0;}
    I2C_Start();//if(!I2C_Start())return 0;
    I2C_SendByte(SlaveAddress+1);
	
    I2C_WaitAck();//if(!I2C_Start())return 0;
    REG_data = I2C_ReadByte();
    I2C_NoAck();
    I2C_Stop();
    return REG_data;
}
/******************************************************************************
/ 函数功能:单字节写入
/ 修改日期:none
/ 输入参数:
/   @arg SlaveAddress   从器件地址
/   @arg REG_Address    寄存器地址
/   @arg REG_data       欲写入的字节数据
/ 输出参数: 读出的字节数据
/ 使用说明:这时一个完整的单字节读取函数
******************************************************************************/
uint8_t Single_Write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)
{
    if(!I2C_Start())return 0;
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return 0;}
    I2C_SendByte(REG_Address );   //设置低起始地址      
    if(!I2C_WaitAck()){I2C_Stop(); return 0;}//I2C_WaitAck();  
    I2C_SendByte(REG_data);
    if(!I2C_WaitAck()){I2C_Stop(); return 0;}//I2C_WaitAck();   
    I2C_Stop(); 
    return 1;
}



