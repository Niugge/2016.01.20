#ifndef __LED_H
#define __LED_H	 

#include "stm32f10x.h"
#include "sys.h"

//Mini STM32开发板
//LED驱动代码			 
//LED端口定义
#define LED0  PAout(8)// PA8
#define LED1  PDout(2)// PD2	
#define LED2  PCout(2)

void LED_Init(void);	//初始化
void red_ON(void);
void red_OFF(void);
void red_toggle(void);
void yellow_ON(void);
void yellow_OFF(void);
void yellow_toggle(void);
void all_ON(void);
void all_OFF(void);
void all_toggle(void);	
void blue_On(void);
void blue_Off(void);
void blue_twinkle(void);
#endif
