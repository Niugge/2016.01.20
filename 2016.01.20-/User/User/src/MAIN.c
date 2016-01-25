/****************************
development board for quard_copter
include MPU6050 HMC5883L MS5611 & GPS
****************************/
#include "stm32f10x.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "flash.h"
#include "eeprom.h"
#include "DATA_Transfer.h"
#include "led.h"
#include "key.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "ms5611.h"
#include "adc.h"
#include "moto.h"
#include "tim_pwm_in.h"
#include "timer.h"
#include "spi.h"
#include "nrf24l01.h"
#include "IMU.h"
#include "filter.h"
#include "ADNS3080.h"
#include "matrix.h"
/****************************���ת��************************

			1:˳ʱ��    2����ʱ��      3��˳ʱ��      4����ʱ��

*************************************************************/
/*********************
��ص�ѹ��⣺PB0
KEY         : PC1 PC13
LED			    : PA8	PD2	PC2
��̬������  ��PC12��SCL��PC11(SDA)(���)
							PB13 SDA,  PB15  SCL(С��)
GPS         : PA9(RX)��PA10(TX) ����ռ���ȼ�3�������ȼ�3
���PWM��� ��PA0,PA1,PA2 ,PA3��TIM5��
ң��PWM���� ��PB6�� PB7��PB8�� PB9 (TIM4)����ռ���ȼ�0�������ȼ�1
							PA15��PB3, PB10��PB11(TIM2): ��ռ���ȼ�0�������ȼ�1
NRF2401     ��SPI1 PA4(CE) PA5(SCK) PA6(MISO) PA7(MOSI) PC4(CS) PC5(IRQ)
��ʱ���ж�  ��TIM3 �ж�����2ms(500HZ) ����ռ���ȼ�0�������ȼ�3
����������  ��PC6  CS,PC7  MISO,PC8  MOSI,PC9 SCLK
*********************/
int main(void)
{
	/******ϵͳ��ʼ��******/
	SystemInit();		 //ϵͳʱ�ӵȳ�ʼ��
	delay_init();	     //��ʱ������ʼ��	  
	NVIC_Configuration();//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(9600);	 //GPS���ڳ�ʼ��Ϊ9600
	LED_Init();			 //LED��ʼ�� PA8 PD2 PC2-3	
	KEY_Init();			 //ȥ����Ư����
#ifdef USE_BATTERY_CHARGE
	Adc_cfig();			 //PB0��ص�ѹ���
#endif
	MPU6050_Init();		 //  mpu6050��ʼ��
	delay_ms(500);
	
			/********************************************************/
	ADNS3080_Init();
  delay_ms(10); 
	printf("Hello World!\r\n");
	printf("ID:%d\r\n",read_register(Product_ID));
  printf("Frame Rate:%d\r\n",read_zhenlv());//��3000֡/�룩24000000/3000֡ÿ��=8000 4800��5000֡
  delay_ms(10);
  printf("1600=%d\n",read_register(Motion)&0x01);//����1��Ϊ1600�ֱ���  
  delay_ms(10);
  printf("Average Pixel:%f\r\n",read_average_pixel()); //ƽ������
	printf("Maximum Pixel:%d\r\n",read_Maximum_pixel()); //ƽ������
			/***********************************************************/
	while(MPU6050_ReadByte(MPU6050_Addr, WHO_AM_I) != 0X68)
	{
		all_OFF();		 //GY_86ģ������ʧ�ܣ�Ϩ��ָʾ
	}
	HMC5883L_Init();	 //HMC5883L��ʼ��
	MS561101BA_Init();	 //MS5611��ʼ��
	Spi1_Init();		 //SPI1��ʼ��
	Nrf24l01_Init(MODEL_TX2,40);
	delay_ms(10);
	while (!Nrf24l01_Check())
	{
		all_ON();		 //2401����ʧ�ܣ���״̬��
	}	
	FLASH_Unlock();
	EE_INIT();
	EE_READ_ACC_OFFSET();
	EE_READ_GYRO_OFFSET();
	EE_READ_CMP_OFFSET();
	EE_READ_BARO_OFFSET();
	EE_READ_PID();	
#ifdef CONTROL_USE_RC	
	Tim_Pwm_In_Init();   //ң��PWM�ɼ�����ȡ4ͨ��ֵTIM4
#endif
	Moto_Init(); 	 	 //TIM2��PWM out ����Ƶ��PWMƵ��=72000/900=8Khz
	Timerx_Init(59,7199);//TIM3 IRQ������������10Khz�ļ���Ƶ�ʣ�������100Ϊ10ms 
	
	LPF2pSetCutoffFreq(1000.0f/6.0f,5.0f);  //����Butterworsec������sample_freq=1000.0f/6��cutoff_freq=30.0f
	/******main loop******/
	while(1)
	{		
#ifdef  CONTROL_USE_RC	
		RC_Read(&Rc_D);	
#endif
		/*********scan key**********/
		Calibration_Sensor();
#ifdef  USE_BATTERY_CHARGE
		bettary = (float )((read_adc(ADC_Channel_8,1)*3.3/4096)*4);//bettery chack
		if(bettary < bettary_low)
		{
			/**�ɻ�ʧ�ر����������	**/						
		}
#endif	
		/****��ȡqiyaji����****/
		 MS5611_Read(&BaroAlt);		//����ִ����Ҫ20msʱ�䣬ִ��Ƶ�ʲ���̫��
		/*****ADNS3080******/
			Read_Data_burst(); //�ñ�����
	}
}




