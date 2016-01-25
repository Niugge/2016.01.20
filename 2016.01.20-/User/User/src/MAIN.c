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
/****************************电机转向************************

			1:顺时针    2：逆时针      3：顺时针      4：逆时针

*************************************************************/
/*********************
电池电压检测：PB0
KEY         : PC1 PC13
LED			    : PA8	PD2	PC2
姿态传感器  ：PC12（SCL）PC11(SDA)(大板)
							PB13 SDA,  PB15  SCL(小板)
GPS         : PA9(RX)，PA10(TX) ：抢占优先级3，子优先级3
电机PWM输出 ：PA0,PA1,PA2 ,PA3（TIM5）
遥控PWM输入 ：PB6， PB7，PB8， PB9 (TIM4)：抢占优先级0，子优先级1
							PA15，PB3, PB10，PB11(TIM2): 抢占优先级0，子优先级1
NRF2401     ：SPI1 PA4(CE) PA5(SCK) PA6(MISO) PA7(MOSI) PC4(CS) PC5(IRQ)
定时器中断  ：TIM3 中断周期2ms(500HZ) ：抢占优先级0，子优先级3
光流传感器  ：PC6  CS,PC7  MISO,PC8  MOSI,PC9 SCLK
*********************/
int main(void)
{
	/******系统初始化******/
	SystemInit();		 //系统时钟等初始化
	delay_init();	     //延时函数初始化	  
	NVIC_Configuration();//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(9600);	 //GPS串口初始化为9600
	LED_Init();			 //LED初始化 PA8 PD2 PC2-3	
	KEY_Init();			 //去除零漂按键
#ifdef USE_BATTERY_CHARGE
	Adc_cfig();			 //PB0电池电压检测
#endif
	MPU6050_Init();		 //  mpu6050初始化
	delay_ms(500);
	
			/********************************************************/
	ADNS3080_Init();
  delay_ms(10); 
	printf("Hello World!\r\n");
	printf("ID:%d\r\n",read_register(Product_ID));
  printf("Frame Rate:%d\r\n",read_zhenlv());//（3000帧/秒）24000000/3000帧每秒=8000 4800则5000帧
  delay_ms(10);
  printf("1600=%d\n",read_register(Motion)&0x01);//等于1则为1600分辨率  
  delay_ms(10);
  printf("Average Pixel:%f\r\n",read_average_pixel()); //平均像素
	printf("Maximum Pixel:%d\r\n",read_Maximum_pixel()); //平均像素
			/***********************************************************/
	while(MPU6050_ReadByte(MPU6050_Addr, WHO_AM_I) != 0X68)
	{
		all_OFF();		 //GY_86模块启动失败，熄灯指示
	}
	HMC5883L_Init();	 //HMC5883L初始化
	MS561101BA_Init();	 //MS5611初始化
	Spi1_Init();		 //SPI1初始化
	Nrf24l01_Init(MODEL_TX2,40);
	delay_ms(10);
	while (!Nrf24l01_Check())
	{
		all_ON();		 //2401启动失败，亮状态灯
	}	
	FLASH_Unlock();
	EE_INIT();
	EE_READ_ACC_OFFSET();
	EE_READ_GYRO_OFFSET();
	EE_READ_CMP_OFFSET();
	EE_READ_BARO_OFFSET();
	EE_READ_PID();	
#ifdef CONTROL_USE_RC	
	Tim_Pwm_In_Init();   //遥控PWM采集，读取4通道值TIM4
#endif
	Moto_Init(); 	 	 //TIM2，PWM out 不分频。PWM频率=72000/900=8Khz
	Timerx_Init(59,7199);//TIM3 IRQ（心跳函数）10Khz的计数频率，计数到100为10ms 
	
	LPF2pSetCutoffFreq(1000.0f/6.0f,5.0f);  //设置Butterworsec参数，sample_freq=1000.0f/6，cutoff_freq=30.0f
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
			/**飞机失控保护代码添加	**/						
		}
#endif	
		/****获取qiyaji数据****/
		 MS5611_Read(&BaroAlt);		//任务执行需要20ms时间，执行频率不能太快
		/*****ADNS3080******/
			Read_Data_burst(); //用爆发读
	}
}




