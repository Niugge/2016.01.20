#include "HMC5883L.h"

#define HEAD_LPF_FACTOR 20

/******************************************************************************
/ ��������:��ʼ��HMC5883
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
******************************************************************************/
void  HMC5883L_Init(void)
{
   Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x14);   //30Hz
   Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x00);   //��������ģʽ
}

/******************************************************************************
/ ��������:��ȡHMC5883�����ݣ���λ���ʻ�
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:none
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
	/****************��0��1***************/	
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
/ ��������:HMC5883У׼
/ �޸�����:none
/ �������:none
/ �������:none
/ ʹ��˵��:����-�ж�-��ѯ (��ѯ)
******************************************************************************/
void HMC5883L_Calibrate(void)
{
   Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x15);   //30Hz,�����Լ�ģʽ
   Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x01);   //��һ����ģʽ
   delay_ms(10);
   Single_Write(HMC5883L_Addr,HMC5883L_REGA,0x14);
   Single_Write(HMC5883L_Addr,HMC5883L_MODE,0x00);   //�ص�����ģʽ
}

/******************************************************************************
/ ��������:���ֽ�д��
/ �޸�����:none
/ �������:
/   @arg SlaveAddress   ��������ַ
/   @arg REG_Address    �Ĵ�����ַ
/ �������: �������ֽ�����
/ ʹ��˵��:��ʱһ�������ĵ��ֽڶ�ȡ����
******************************************************************************/
uint8_t Single_Read(uint8_t SlaveAddress,uint8_t REG_Address)
{   
    uint8_t REG_data;       
    I2C_Start();//if(!I2C_Start())return 0
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//���ø���ʼ��ַ+������ַ 
    I2C_WaitAck();//if(!I2C_WaitAck()){I2C_Stop();return 0;}
    I2C_SendByte((uint8_t) REG_Address);   //���õ���ʼ��ַ      
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
/ ��������:���ֽ�д��
/ �޸�����:none
/ �������:
/   @arg SlaveAddress   ��������ַ
/   @arg REG_Address    �Ĵ�����ַ
/   @arg REG_data       ��д����ֽ�����
/ �������: �������ֽ�����
/ ʹ��˵��:��ʱһ�������ĵ��ֽڶ�ȡ����
******************************************************************************/
uint8_t Single_Write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)
{
    if(!I2C_Start())return 0;
    I2C_SendByte(SlaveAddress);   //�����豸��ַ+д�ź�//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//���ø���ʼ��ַ+������ַ 
    if(!I2C_WaitAck()){I2C_Stop(); return 0;}
    I2C_SendByte(REG_Address );   //���õ���ʼ��ַ      
    if(!I2C_WaitAck()){I2C_Stop(); return 0;}//I2C_WaitAck();  
    I2C_SendByte(REG_data);
    if(!I2C_WaitAck()){I2C_Stop(); return 0;}//I2C_WaitAck();   
    I2C_Stop(); 
    return 1;
}



