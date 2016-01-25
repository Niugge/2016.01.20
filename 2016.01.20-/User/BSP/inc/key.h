#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#include "delay.h"	
#include "IMU.h"
  	
//#define KEY1  PAin(15)	 
//#define WK_UP PAin(0)	 
 

//#define KEY1  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)//读取按键1
//#define WK_UP GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)//读取按键2 
#define KEY0  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1)//读取按键0
#define KEY1  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13)//读取按键1
 
 
#define KEY1_PRES	1		//KEY1 
#define KEY2_PRES	2		//KEY2 
#define WKUP_PRES	3		//WK_UP  

void KEY_Init(void);	//IO初始化
uint8_t KEY_Scan(uint8_t mode); //按键扫描函数	
void  Calibration_Sensor(void);
#endif
