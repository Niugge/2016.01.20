#include "MPU6050.h"
#include "stm32f10x_gpio.h"
#include "IMU.h"

/**MPU6050ԭʼ���ݴ��**/
s16 Temperature; 	//6050�����¶�ֵ

static uint8_t	mpu6050_buffer[14];	//iic��ȡ��������
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(void) 
{
	uint8_t data = 0;	
	data = MPU6050_ReadByte(MPU6050_Addr,MPU6050_RA_USER_CTRL);
	data = data & 0XDF;
	MPU6050_WriteByte(MPU6050_Addr,MPU6050_RA_USER_CTRL,data);	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*��������:	    ���� MPU6050 �Ƿ�ΪAUX I2C�ߵ�����
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(void)
{
	uint8_t data = 0;	
	data = MPU6050_ReadByte(MPU6050_Addr,MPU6050_RA_INT_PIN_CFG);
	data = data | 0X02;
	MPU6050_WriteByte(MPU6050_Addr,MPU6050_RA_USER_CTRL,data);
}


/****SCL PC12 SDA PC11*****/
void MPU6050_Init(void)
{
	 GPIO_InitTypeDef  GPIO_InitStructure;
	/**************��ߴ翪������̬����������****************/		
	 #ifdef USE_BIG_SIZE_BOARD	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 
	 //PC12 SCL
	 GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;				 
	 GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP; 		 
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOC, &GPIO_InitStructure);
	 GPIO_SetBits(GPIOC,GPIO_Pin_12);						
		//PC11 SDA
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	    		
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	 GPIO_Init(GPIOC, &GPIO_InitStructure);	
	 GPIO_SetBits(GPIOC,GPIO_Pin_11);	
	 #endif
	/**************С�ߴ翪������̬����������****************/
	 #ifdef USE_LITTLE_SIZE_BOARD
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 
	 //PB15 SCL
	 GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;				 
	 GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP; 		 
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOB, &GPIO_InitStructure);
	 GPIO_SetBits(GPIOB,GPIO_Pin_15);						
		//PB13 SDA
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;	    		
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD; 		
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	 GPIO_Init(GPIOB, &GPIO_InitStructure);	
	 GPIO_SetBits(GPIOB,GPIO_Pin_13);
	 #endif	 
	/*************��̬ģ���ʼ��*************/       //���ٶȼƺ������Ƕ���ʹ��1000Hz����
	MPU6050_WriteByte(MPU6050_Addr, PWR_MGMT_1, 0x00);	 //�������״̬
	MPU6050_WriteByte(MPU6050_Addr, SMPLRT_DIV, 0x00);    //SMPLRT_DIV    -- SMPLRT_DIV = 3��  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	MPU6050_WriteByte(MPU6050_Addr, MPU_CONFIG, 0x03);    //Ƭ�ڼĴ������õĵ�ͨ�˲���ʹ�ô���42Hz
	MPU6050_WriteByte(MPU6050_Addr, GYRO_CONFIG, 0x18);  // ����2000/s
	MPU6050_WriteByte(MPU6050_Addr, ACCEL_CONFIG, 0x00); // ����2g��0x08��Ӧ4g
	MPU6050_setI2CMasterModeEnabled();
	MPU6050_setI2CBypassEnabled();
	MPU6050_WriteByte(MPU6050_Addr,INT_PIN_CFG, 0x42);   //ʹ����·I2C
	MPU6050_WriteByte(MPU6050_Addr,USER_CTRL, 0x40);     //ʹ����·I2C
}

 void MPU6050_Read(MPU6050_DATA *sensor_data)
{
		// �¶�
		mpu6050_buffer[0] = MPU6050_ReadByte(MPU6050_Addr, TEMP_OUT_L); 
		mpu6050_buffer[1] = MPU6050_ReadByte(MPU6050_Addr, TEMP_OUT_H);
		Temperature = (mpu6050_buffer[1] << 8) | mpu6050_buffer[0]; //16λ����		
		// ������
		mpu6050_buffer[2] = MPU6050_ReadByte(MPU6050_Addr, GYRO_XOUT_L); 
		mpu6050_buffer[3] = MPU6050_ReadByte(MPU6050_Addr, GYRO_XOUT_H);
		sensor_data->gx = (mpu6050_buffer[3] << 8) | mpu6050_buffer[2];
		sensor_data->gx -= MPU6050_OFFSER.gx ;//
		
		mpu6050_buffer[4] = MPU6050_ReadByte(MPU6050_Addr, GYRO_YOUT_L);
		mpu6050_buffer[5] = MPU6050_ReadByte(MPU6050_Addr, GYRO_YOUT_H);
		sensor_data->gy = (mpu6050_buffer[5] << 8) | mpu6050_buffer[4];
		sensor_data->gy -= MPU6050_OFFSER.gy;//

		mpu6050_buffer[6] = MPU6050_ReadByte(MPU6050_Addr, GYRO_ZOUT_L);
		mpu6050_buffer[7] = MPU6050_ReadByte(MPU6050_Addr, GYRO_ZOUT_H);
		sensor_data->gz = (mpu6050_buffer[7] << 8) | mpu6050_buffer[6];
		sensor_data->gz -= MPU6050_OFFSER.gz;//
	
	// ���ٶȼ�
		mpu6050_buffer[8] = MPU6050_ReadByte(MPU6050_Addr, ACCEL_XOUT_L); 
		mpu6050_buffer[9] = MPU6050_ReadByte(MPU6050_Addr, ACCEL_XOUT_H);
		sensor_data->ax = (mpu6050_buffer[9] << 8) | mpu6050_buffer[8];
		sensor_data->ax -= MPU6050_OFFSER.ax;//
		
		mpu6050_buffer[10] = MPU6050_ReadByte(MPU6050_Addr, ACCEL_YOUT_L);
		mpu6050_buffer[11] = MPU6050_ReadByte(MPU6050_Addr, ACCEL_YOUT_H);
		sensor_data->ay = (mpu6050_buffer[11] << 8) | mpu6050_buffer[10];
		sensor_data->ay -= MPU6050_OFFSER.ay;//

		mpu6050_buffer[12]=MPU6050_ReadByte(MPU6050_Addr, ACCEL_ZOUT_L);
		mpu6050_buffer[13]=MPU6050_ReadByte(MPU6050_Addr, ACCEL_ZOUT_H);
		sensor_data->az = (mpu6050_buffer[13] << 8) | mpu6050_buffer[10];
		sensor_data->az -= MPU6050_OFFSER.az;//
		/******ȥ����Ư*********/
		if(GYRO_OFFSET_OK)
	{
		static int32_t	tempgx=0,tempgy=0,tempgz=0;
		static uint8_t cnt_g=0;
		static int i = 0;
		if(cnt_g==0)
		{
			MPU6050_OFFSER.gx=0;
			MPU6050_OFFSER.gy=0;
			MPU6050_OFFSER.gz=0;			
			tempgx = 0;
			tempgy = 0;
			tempgz = 0;
			cnt_g = 1;
			return;
		}
		tempgx+= sensor_data->gx;
		tempgy+= sensor_data->gy;
		tempgz+= sensor_data->gz;
		if(cnt_g==200)
		{
			MPU6050_OFFSER.gx = tempgx/cnt_g;//49
			MPU6050_OFFSER.gy = tempgy/cnt_g;//14
			MPU6050_OFFSER.gz = tempgz/cnt_g;//-10
			cnt_g = 0;
			GYRO_OFFSET_OK = 0;
			EE_SAVE_GYRO_OFFSET();//��������
			for(i=0;i<3;i++)
				{all_ON(); delay_ms(300);all_OFF();delay_ms(300);}
			return;
		}
		cnt_g++;
	}
	
	if(ACC_OFFSET_OK)
	{
		static int32_t	tempax=0,tempay=0,tempaz=0;
		static uint8_t cnt_a=0;
		static int i =0;
		if(cnt_a==0)
		{
			MPU6050_OFFSER.ax = 0;
			MPU6050_OFFSER.ay = 0;
			MPU6050_OFFSER.az = 0;
			tempax = 0;
			tempay = 0;
			tempaz = 0;
			cnt_a = 1;
			return;
		}
		tempax+= sensor_data->ax;
		tempay+= sensor_data->ay;
		tempaz+= (sensor_data->az - 16384);
		if(cnt_a==200)
		{
			MPU6050_OFFSER.ax = tempax/cnt_a;
			MPU6050_OFFSER.ay = tempay/cnt_a;
			MPU6050_OFFSER.az = tempaz/cnt_a;
			cnt_a = 0;
			ACC_OFFSET_OK = 0;
			EE_SAVE_ACC_OFFSET();//��������
			for(i=0;i<3;i++)
			{all_ON(); delay_ms(300);all_OFF();delay_ms(300);}
			return;
		}
		cnt_a++;		
	}
}	
		 
/*��������Ż��ٶ�,��������͵�5����д��*/
void I2C_delay(void)
{
		
   vu8 i=1; //30us,��Ϳ��Ե�5us
   while(i) 
   { 
     i--; 
   }  
}	
	
/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather	 Start
****************************************************************************** */
uint8_t I2C_Start(void)
{
	SDA_H;
	SCL_H;
	I2C_delay();
	if(!SDA_read)return 0;	//SDA��Ϊ�͵�ƽ������æ,�˳�
	SDA_L;
	I2C_delay();
	if(SDA_read) return 0;	//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
	SDA_L;
	I2C_delay();
	return 1;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Stop(void)
{
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SDA_H;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Ack(void)
{	
	SCL_L;
	I2C_delay();
	SDA_L;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)
{	
	SCL_L;
	I2C_delay();
	SDA_H;
	I2C_delay();
	SCL_H;
	I2C_delay();
	SCL_L;
	I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather	 Reserive Slave Acknowledge Single
****************************************************************************** */
uint8_t I2C_WaitAck(void) 	 //����Ϊ:=1��ACK,=0��ACK
{
	SCL_L;
	I2C_delay();
	SDA_H;			
	I2C_delay();
	SCL_H;
	I2C_delay();
	if(SDA_read)
	{
      SCL_L;
	  I2C_delay();
      return 0;
	}
	SCL_L;
	I2C_delay();
	return 1;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(uint8_t SendByte) //���ݴӸ�λ����λ//
{
    uint8_t i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        I2C_delay();
		SCL_H;
        I2C_delay();
    }
    SCL_L;
}  
/*******************************************************************************
* Function Name  : I2C_ReadByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
uint8_t I2C_ReadByte(void)  //���ݴӸ�λ����λ//
{ 
    uint8_t i=8;
    uint8_t ReceiveByte=0;
    SDA_H;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L;
      I2C_delay();
	  SCL_H;
      I2C_delay();	
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;
    return ReceiveByte;
} 
//ZRX          
/********************���ֽ�д��***********************/
uint8_t MPU6050_WriteByte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)		   
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

/***********************���ֽڶ�ȡ******************/
uint8_t MPU6050_ReadByte(uint8_t SlaveAddress,uint8_t REG_Address)
{   
	unsigned char REG_data;     	
	I2C_Start();//if(!I2C_Start())return 0
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//���ø���ʼ��ַ+������ַ 
    I2C_WaitAck();//if(!I2C_WaitAck()){I2C_Stop();test=1; return 0;}
    I2C_SendByte((uint8_t) REG_Address);   //���õ���ʼ��ַ      
    I2C_WaitAck();  //if(!I2C_WaitAck()){I2C_Stop(); return 0;}
		
    I2C_Start();//if(!I2C_Start())return 0;
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();//if(!I2C_WaitAck()){I2C_Stop(); return 0;}
    REG_data= I2C_ReadByte();
    I2C_NoAck();
    I2C_Stop();
	return REG_data;
}						      

