#include "moto.h"

void Moto_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	//使能电机用的时钟,使用复用功能始终
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO , ENABLE); 

	//管脚冲映射
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE); //Timer2完全重映射  
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //JTAG管脚冲映射
	//设置电机使用到得管脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 ; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//tim使能
	Tim5_init();	
}

void Moto_PwmRflash(MOTO_PWM *moto_pwm)
{		
	/****************限幅处理**********************/
	if(moto_pwm->MOTO1_PWM > Moto_PwmMax)	moto_pwm->MOTO1_PWM = Moto_PwmMax;
	if(moto_pwm->MOTO2_PWM > Moto_PwmMax)	moto_pwm->MOTO2_PWM = Moto_PwmMax;
	if(moto_pwm->MOTO3_PWM > Moto_PwmMax)	moto_pwm->MOTO3_PWM = Moto_PwmMax;
	if(moto_pwm->MOTO4_PWM > Moto_PwmMax)	moto_pwm->MOTO4_PWM = Moto_PwmMax;
	
	if(moto_pwm->MOTO1_PWM < Moto_PwmMin)	moto_pwm->MOTO1_PWM = Moto_PwmMin;
	if(moto_pwm->MOTO2_PWM < Moto_PwmMin)	moto_pwm->MOTO2_PWM = Moto_PwmMin;
	if(moto_pwm->MOTO3_PWM < Moto_PwmMin)	moto_pwm->MOTO3_PWM = Moto_PwmMin;
	if(moto_pwm->MOTO4_PWM < Moto_PwmMin)	moto_pwm->MOTO4_PWM = Moto_PwmMin;
	
	TIM5->CCR1 = moto_pwm->MOTO1_PWM;
	TIM5->CCR2 = moto_pwm->MOTO2_PWM;
	TIM5->CCR3 = moto_pwm->MOTO3_PWM;
	TIM5->CCR4 = moto_pwm->MOTO4_PWM;
}

void Tim5_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	uint16_t PrescalerValue = 0;
	/* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
    clock at 24 MHz the Prescaler is computed as following:
	- Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices
	
    The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
	= 24 MHz / 1000 = 24 KHz
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
	----------------------------------------------------------------------- */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) 72 - 1;//(SystemCoreClock / 24000000) - 1
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;		//计数上线	
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;	//pwm时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	/* PWM1 Mode configuration: Channel */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;//初始占空比为0
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	/* PWM1 Mode configuration: 4 Channel */
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);	
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_Cmd(TIM5, ENABLE);
}




