#include "ms5611.h"

#define P0 1013.25f		//标准大气压

uint16_t Cal_C[7];  	//用于存放PROM中的8组数据	
uint8_t Cal_Num = 50;	// calibration
/*=========================================================
************************MS561101BA程序*********************
=========================================================*/
void MS561101BA_Init(void)
{
	while(MS561101BA_RESET()!=1);
	delay_ms(100);
	MS561101BA_PROM_READ();
	delay_ms(100);	
} 

uint8_t MS561101BA_RESET(void)
{	
  if(!I2C_Start())return 0;
  I2C_SendByte(MS561101BA_SlaveAddress); 
  if(!I2C_WaitAck()){I2C_Stop();test=1; return 0;}
  I2C_SendByte((uint8_t) MS561101BA_RST);        
  I2C_WaitAck();
	I2C_Stop();
	return 1;
}

uint16_t MS5611_ReadByte_16b(uint8_t SlaveAddress,uint8_t REG_Address)
{
	unsigned char REG_data1,REG_data2;
	uint16_t REG_data;
	if(!I2C_Start())return 0;
	I2C_SendByte(SlaveAddress); 
	if(!I2C_WaitAck()){I2C_Stop();test=1; return 0;}
	I2C_SendByte((uint8_t) REG_Address);  
	I2C_WaitAck();
	I2C_Start();
	I2C_SendByte(SlaveAddress+1);
	I2C_WaitAck();
	REG_data1= I2C_ReadByte();
	I2C_Ack();
	REG_data2= I2C_ReadByte();
	I2C_NoAck();	
	I2C_Stop();
	REG_data = REG_data1<<8|REG_data2;
	return REG_data;
}

uint8_t MS5611_ReadByte_8b(uint8_t SlaveAddress,uint8_t REG_Address)
{
	unsigned char REG_data;     	
	if(!I2C_Start())return 0;
	I2C_SendByte(SlaveAddress); 
	if(!I2C_WaitAck()){I2C_Stop();test=1; return 0;}
	I2C_SendByte((uint8_t) REG_Address);  
	I2C_WaitAck();
	I2C_Start();
	I2C_SendByte(SlaveAddress+1);
	I2C_WaitAck();
	REG_data= I2C_ReadByte();
	I2C_NoAck();
	I2C_Stop();
	return REG_data;
}


/********从PROM读取出厂校准数据**********/
void MS561101BA_PROM_READ(void)
{	
	Cal_C[1] = MS5611_ReadByte_16b(MS561101BA_SlaveAddress,MS5611_PROM_COEFF_1);
	Cal_C[2] = MS5611_ReadByte_16b(MS561101BA_SlaveAddress,MS5611_PROM_COEFF_2);
	Cal_C[3] = MS5611_ReadByte_16b(MS561101BA_SlaveAddress,MS5611_PROM_COEFF_3);
	Cal_C[4] = MS5611_ReadByte_16b(MS561101BA_SlaveAddress,MS5611_PROM_COEFF_4);
	Cal_C[5] = MS5611_ReadByte_16b(MS561101BA_SlaveAddress,MS5611_PROM_COEFF_5);
	Cal_C[6] = MS5611_ReadByte_16b(MS561101BA_SlaveAddress,MS5611_PROM_COEFF_6);
	//打印PROM读取出厂校准数据，检测数据传输是否正常
//	printf("c1:%d\r\n",Cal_C[1]);
//	printf("c2:%d\r\n",Cal_C[2]);
//	printf("c3:%d\r\n",Cal_C[3]);
//	printf("c4:%d\r\n",Cal_C[4]);
//	printf("c5:%d\r\n",Cal_C[5]);
//	printf("c6:%d\r\n",Cal_C[6]);
}
/***********24bit ADC*******************/
u32 MS561101BA_ADC_CONVERSION(uint8_t command)
{
	u32 conversion=0;
	u32 conv1,conv2,conv3; 
	
	I2C_Start();//if(!I2C_Start())return 0;
	I2C_SendByte(MS561101BA_SlaveAddress);//
	I2C_WaitAck();//if(!I2C_WaitAck()){I2C_Stop();test=1; return 0;}
	I2C_SendByte(command);	//0x48,0x58
	I2C_WaitAck();//if(!I2C_WaitAck()){I2C_Stop();test=1; return 0;}
	I2C_Stop();
	//必须加延时，不然读不到数据
	delay_ms(10);

	I2C_Start();//if(!I2C_Start())return 0;
	I2C_SendByte(MS561101BA_SlaveAddress);
	I2C_WaitAck();//if(!I2C_WaitAck()){I2C_Stop();test=1; return 0;}
	I2C_SendByte(MS5611_ADC);//0x00
	I2C_WaitAck();//if(!I2C_WaitAck()){I2C_Stop();test=1; return 0;}
	I2C_Stop();

	I2C_Start();//if(!I2C_Start())return 0;
	I2C_SendByte(MS561101BA_SlaveAddress+1);
	I2C_WaitAck();//if(!I2C_WaitAck()){I2C_Stop();test=1; return 0;}
	
	conv1=I2C_ReadByte();
	I2C_Ack();
	conv2=I2C_ReadByte();
	I2C_Ack();
	conv3=I2C_ReadByte();
	I2C_NoAck();
	I2C_Stop();

	conversion= (conv1<<16) + (conv2<<8) + conv3;
	return conversion;
}

/******************数据转换*******************/
void MS5611_Read(vs32 *alt)
{    
	int32_t pressure,dT;
	int64_t off = 0;
	int64_t sens = 0;
	uint32_t ms5611_ut;  // static result of temperature measurement
	uint32_t ms5611_up;  // static result of pressure measurement	
	int32_t temperature, off2 = 0, sens2 = 0, delt = 0;

	ms5611_ut = MS561101BA_ADC_CONVERSION(MS561101BA_D2_OSR_4096);
	ms5611_up = MS561101BA_ADC_CONVERSION(MS561101BA_D1_OSR_4096);
	
	dT = ms5611_ut - ((uint32_t)Cal_C[5] << 8);
	off = ((uint32_t)Cal_C[2] << 16) + (((int64_t)dT * Cal_C[4]) >> 7);
	sens = ((uint32_t)Cal_C[1] << 15) + (((int64_t)dT * Cal_C[3]) >> 8);
	temperature = 2000 + (((int64_t)dT * Cal_C[6]) >> 23);
	/**temperature lower than 20degC **/
	if (temperature < 2000) 
	{  
			delt = temperature - 2000;
			delt = delt * delt;
			off2 = (5 * delt) >> 1;
			sens2 = (5 * delt) >> 2;
	/**temperature lower than -15degC**/ 
			if (temperature < -1500) 
			{ 
				delt = temperature + 1500;
				delt = delt * delt;
				off2  += 7 * delt;
				sens2 += (11 * delt) >> 1;
			}
	}
	off  -= off2; 
	sens -= sens2;
	pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
	
	pressure = (int)((1.0f - pow(pressure / 101325.0f, 0.190295f)) * 4433000.0f); // centimeter
	*alt = pressure-BaroOffset;
	
	if(MS_OFFSET_OK)
	{
		if(sum_cnt)
			{
				sum_cnt--;
				sum_temp += *alt;
				if(sum_cnt == 0)
				{
					static uint8_t i = 0;
					BaroOffset = sum_temp / BARO_CAL_CNT;
					Alt_Estimated=0;
					for(i=0;i<3;i++)
						{all_ON(); delay_ms(300);all_OFF();delay_ms(300);}		
					MS_OFFSET_OK = 0;
					EE_SAVE_BARO_OFFSET();
				}				
			}		
	}	
} 



