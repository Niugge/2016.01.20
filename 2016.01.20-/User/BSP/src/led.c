#include "led.h"
#include "delay.h"


//��ʼ��PA8��PD2Ϊ�����.��ʹ���������ڵ�ʱ��		    
//LED IO��ʼ��
void LED_Init(void)
{
	 GPIO_InitTypeDef  GPIO_InitStructure;
		
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //ʹ��PA�˿�ʱ��	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 //LED0-->PA.8 �˿�����
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 GPIO_SetBits(GPIOA,GPIO_Pin_8);						 //PA.8 �����

	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	 //ʹ��PC�˿�ʱ��	
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;	 //LED0-->PA.8 �˿�����
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //�������
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOC, &GPIO_InitStructure);
	 GPIO_SetBits(GPIOC,GPIO_Pin_2|GPIO_Pin_3);
	 
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);   //ʹ��PD�˿�ʱ��

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	    		 //LED1-->PD.2 �˿�����
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	 GPIO_Init(GPIOD, &GPIO_InitStructure);	
	 GPIO_SetBits(GPIOD,GPIO_Pin_2);	//PD.2 ����� 

	red_ON();
	yellow_OFF();
	delay_ms(100);
	red_OFF();
	yellow_ON();
	delay_ms(100);
	red_ON();
	yellow_ON();
	delay_ms(100);
}

void red_ON(void)
{
	LED0  = 0;
}

void red_OFF(void)
{
	LED0 = 1;
}
void red_toggle(void)
{
	LED0 = !LED0;
}

void yellow_ON(void)
{
	LED1 = 0;
}

void yellow_OFF(void)
{
	LED1 = 1;
}

void yellow_toggle(void)
{
	LED1 = !LED1;
}
void all_ON(void)
{
	LED0 = 0;
	LED1 = 0;
}

void all_OFF(void)
{
	LED0 = 1;
	LED1 = 1;
}


void all_toggle(void)
{
	LED0 = !LED0;
	LED1 = !LED1;
}

void blue_On(void)
{
	LED2 = 0;
}
void blue_Off(void)
{
	LED2 = 1;
}

void blue_twinkle(void)
{
	LED2 = 0;
	delay_ms(300);
	LED2 = 1;
	delay_ms(300);
}


