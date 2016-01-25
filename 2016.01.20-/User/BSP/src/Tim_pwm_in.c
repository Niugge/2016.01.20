#include "tim_pwm_in.h"

#define RC_MAX	2000u
#define RC_MIN	1000u

void Tim_Pwm_In_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  		TIM_ICInitStructure;
	GPIO_InitTypeDef 		GPIO_InitStructure;
	NVIC_InitTypeDef		NVIC_InitStructure;

	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE); //Timer2完全重映射  
	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //JTAG管脚冲映射
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4|RCC_APB1Periph_TIM2,ENABLE);//
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);          //定时器4输入捕获
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);          //定时器2输入捕获
	
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructure.TIM_Period = 0XFFFF;
	/**CLK = 72000000/TIM_Prescaler***/
	TIM_TimeBaseStructure.TIM_Prescaler=71;
	/**set clock division ***/
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	/**count up***/
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
		
	
	/* Enable the CC1-4 Interrupt Request */
	TIM_ITConfig(TIM4, TIM_IT_CC1,ENABLE);//TIM_IT_Update|
	TIM_ITConfig(TIM4, TIM_IT_CC2,ENABLE);//TIM_IT_Update|
	TIM_ITConfig(TIM4, TIM_IT_CC3,ENABLE);//TIM_IT_Update|
	TIM_ITConfig(TIM4, TIM_IT_CC4,ENABLE);//TIM_IT_Update|
		
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 

	/* TIM enable counter */
	TIM_Cmd(TIM4, ENABLE);	
	
	//*******************定时器2设置*************************//
	//TIM_DeInit(TIM2);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE); //Timer2完全重映射  
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //JTAG管脚冲映射
	TIM_TimeBaseStructure.TIM_Period = 0XFFFF;
	/**CLK = 72000000/TIM_Prescaler***/
	TIM_TimeBaseStructure.TIM_Prescaler=71;
	/**set clock division ***/
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	/**count up***/
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
//	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
//	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
//	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
//	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
//	TIM_ICInit(TIM2, &TIM_ICInitStructure);
//	
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
//	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
//	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
//	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
//	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
//	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;	
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;     
	TIM_ICInitStructure.TIM_ICFilter = 0x0;       
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	/* Enable the CC1-4 Interrupt Request */
	//TIM_ITConfig(TIM2, TIM_IT_CC1,ENABLE);//TIM_IT_Update|
	//TIM_ITConfig(TIM2, TIM_IT_CC2,ENABLE);//TIM_IT_Update|   //加上这句话程序运行不下去，就会报错
	TIM_ITConfig(TIM2, TIM_IT_CC3,ENABLE);//TIM_IT_Update|
	TIM_ITConfig(TIM2, TIM_IT_CC4,ENABLE);//TIM_IT_Update|
		
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 

	/* TIM enable counter */
	TIM_Cmd(TIM2, ENABLE);	
}

/** @addtogroup TIM_Input_Capture
  * @{
  */ 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/
/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
__IO uint16_t IC1ReadValue1 = 0, IC1ReadValue2 = 0;
__IO uint16_t CaptureNumber1 = 0;
__IO uint32_t Capture1 = 0;

__IO uint16_t IC2ReadValue1 = 0, IC2ReadValue2 = 0;
__IO uint16_t CaptureNumber2 = 0;
__IO uint32_t Capture2 = 0;

__IO uint16_t IC3ReadValue1 = 0, IC3ReadValue2 = 0;
__IO uint16_t CaptureNumber3 = 0;
__IO uint32_t Capture3 = 0;

__IO uint16_t IC4ReadValue1 = 0, IC4ReadValue2 = 0;
__IO uint16_t CaptureNumber4 = 0;
__IO uint32_t Capture4 = 0;

__IO uint16_t IC5ReadValue1 = 0, IC5ReadValue2 = 0;
__IO uint16_t CaptureNumber5 = 0;
__IO uint32_t Capture5 = 0;

__IO uint16_t IC6ReadValue1 = 0, IC6ReadValue2 = 0;
__IO uint16_t CaptureNumber6 = 0;
__IO uint32_t Capture6 = 0;

__IO uint16_t IC7ReadValue1 = 0, IC7ReadValue2 = 0;
__IO uint16_t CaptureNumber7 = 0;
__IO uint32_t Capture7 = 0;

__IO uint16_t IC8ReadValue1 = 0, IC8ReadValue2 = 0;
__IO uint16_t CaptureNumber8 = 0;
__IO uint32_t Capture8 = 0;

void Tim4_Pwm_In_Irq(void)
{
  if(TIM_GetITStatus(TIM4, TIM_IT_CC1) == SET) 
  {
    /* Clear TIM4 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
    if(CaptureNumber1 == 0)
    {
      /* Get the Input Capture value */
      IC1ReadValue1 = TIM_GetCapture1(TIM4);
	  TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Falling);
      CaptureNumber1 = 1;
    }
    else if(CaptureNumber1 == 1)
    {
      /* Get the Input Capture value */
      IC1ReadValue2 = TIM_GetCapture1(TIM4); 
      TIM_OC1PolarityConfig(TIM4,TIM_ICPolarity_Rising);
      /* Capture computation */
      if (IC1ReadValue2 > IC1ReadValue1)
      {
        Capture1 = (IC1ReadValue2 - IC1ReadValue1); 
      }
      else
      {
        Capture1 = ((0xFFFF - IC1ReadValue1) + IC1ReadValue2); 
      }
      /* Frequency computation */ 
      CaptureNumber1 = 0;
    }
  }

  if(TIM_GetITStatus(TIM4, TIM_IT_CC2) == SET) 
  {
    /* Clear TIM4 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
    if(CaptureNumber2 == 0)
    {
      /* Get the Input Capture value */
      IC2ReadValue1 = TIM_GetCapture2(TIM4);
	  TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Falling);
      CaptureNumber2 = 1;
    }
    else if(CaptureNumber2 == 1)
    {
      /* Get the Input Capture value */
      IC2ReadValue2 = TIM_GetCapture2(TIM4); 
      TIM_OC2PolarityConfig(TIM4,TIM_ICPolarity_Rising);
      /* Capture computation */
      if (IC2ReadValue2 > IC2ReadValue1)
      {
        Capture2 = (IC2ReadValue2 - IC2ReadValue1); 
      }
      else
      {
        Capture2 = ((0xFFFF - IC2ReadValue1) + IC2ReadValue2); 
      }
      /* Frequency computation */ 
      CaptureNumber2 = 0;
    }
  }  
  
  if(TIM_GetITStatus(TIM4, TIM_IT_CC3) == SET) 
  {
    /* Clear TIM4 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
    if(CaptureNumber3 == 0)
    {
      /* Get the Input Capture value */
      IC3ReadValue1 = TIM_GetCapture3(TIM4);
	  TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Falling);
      CaptureNumber3 = 1;
    }
    else if(CaptureNumber3 == 1)
    {
      /* Get the Input Capture value */
      IC3ReadValue2 = TIM_GetCapture3(TIM4); 
      TIM_OC3PolarityConfig(TIM4,TIM_ICPolarity_Rising);
      /* Capture computation */
      if (IC3ReadValue2 > IC3ReadValue1)
      {
        Capture3 = (IC3ReadValue2 - IC3ReadValue1); 
      }
      else
      {
        Capture3 = ((0xFFFF - IC3ReadValue1) + IC3ReadValue2); 
      }
      /* Frequency computation */ 
      CaptureNumber3 = 0;
    }
  }

  if(TIM_GetITStatus(TIM4, TIM_IT_CC4) == SET) 
  {
    /* Clear TIM4 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
    if(CaptureNumber4 == 0)
    {
      /* Get the Input Capture value */
      IC4ReadValue1 = TIM_GetCapture4(TIM4);
	  TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Falling);
      CaptureNumber4 = 1;
    }
    else if(CaptureNumber4 == 1)
    {
      /* Get the Input Capture value */
      IC4ReadValue2 = TIM_GetCapture4(TIM4); 
      TIM_OC4PolarityConfig(TIM4,TIM_ICPolarity_Rising);
      /* Capture computation */
      if (IC4ReadValue2 > IC4ReadValue1)
      {
        Capture4 = (IC4ReadValue2 - IC4ReadValue1); 
      }
      else
      {
        Capture4 = ((0xFFFF - IC4ReadValue1) + IC4ReadValue2); 
      }
      /* Frequency computation */ 
      CaptureNumber4 = 0;
    }
  }  
}

void Tim2_Pwm_In_Irq(void)
{
  if(TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET) 
  {
    /* Clear TIM4 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
    if(CaptureNumber5 == 0)
    {
      /* Get the Input Capture value */
      IC5ReadValue1 = TIM_GetCapture1(TIM2);
	  TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling);
      CaptureNumber5 = 1;
    }
    else if(CaptureNumber5 == 1)
    {
      /* Get the Input Capture value */
      IC5ReadValue2 = TIM_GetCapture1(TIM2); 
      TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Rising);
      /* Capture computation */
      if (IC5ReadValue2 > IC5ReadValue1)
      {
        Capture5 = (IC5ReadValue2 - IC5ReadValue1); 
      }
      else
      {
        Capture5 = ((0xFFFF - IC5ReadValue1) + IC5ReadValue2); 
      }
      /* Frequency computation */ 
      CaptureNumber5 = 0;
    }
  }  
	if(TIM_GetITStatus(TIM2, TIM_IT_CC1) == SET) 
  {
    /* Clear TIM4 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
    if(CaptureNumber6 == 0)
    {
      /* Get the Input Capture value */
      IC6ReadValue1 = TIM_GetCapture1(TIM2);
	  TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling);
      CaptureNumber6 = 1;
    }
    else if(CaptureNumber6 == 1)
    {
      /* Get the Input Capture value */
      IC6ReadValue2 = TIM_GetCapture1(TIM2); 
      TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Rising);
      /* Capture computation */
      if (IC6ReadValue2 > IC6ReadValue1)
      {
        Capture6 = (IC6ReadValue2 - IC6ReadValue1); 
      }
      else
      {
        Capture6 = ((0xFFFF - IC6ReadValue1) + IC6ReadValue2); 
      }
      /* Frequency computation */ 
      CaptureNumber6 = 0;
    }
  } 
	if(TIM_GetITStatus(TIM2, TIM_IT_CC3) == SET) 
  {
    /* Clear TIM4 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
    if(CaptureNumber7 == 0)
    {
      /* Get the Input Capture value */
      IC7ReadValue1 = TIM_GetCapture3(TIM2);
	  TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Falling);
      CaptureNumber7 = 1;
    }
    else if(CaptureNumber7 == 1)
    {
      /* Get the Input Capture value */
      IC7ReadValue2 = TIM_GetCapture3(TIM2); 
      TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Rising);
      /* Capture computation */
      if (IC7ReadValue2 > IC7ReadValue1)
      {
        Capture7 = (IC7ReadValue2 - IC7ReadValue1); 
      }
      else
      {
        Capture7 = ((0xFFFF - IC7ReadValue1) + IC7ReadValue2); 
      }
      /* Frequency computation */ 
      CaptureNumber7 = 0;
    }
  } 
	if(TIM_GetITStatus(TIM2, TIM_IT_CC4) == SET) 
  {
    /* Clear TIM4 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
    if(CaptureNumber8 == 0)
    {
      /* Get the Input Capture value */
      IC8ReadValue1 = TIM_GetCapture4(TIM2);
	  TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Falling);
      CaptureNumber8 = 1;
    }
    else if(CaptureNumber8 == 1)
    {
      /* Get the Input Capture value */
      IC8ReadValue2 = TIM_GetCapture4(TIM2); 
      TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Rising);
      /* Capture computation */
      if (IC8ReadValue2 > IC8ReadValue1)
      {
        Capture8 = (IC8ReadValue2 - IC8ReadValue1); 
      }
      else
      {
        Capture8 = ((0xFFFF - IC8ReadValue1) + IC8ReadValue2); 
      }
      /* Frequency computation */ 
      CaptureNumber8 = 0;
    }
  } 
}

void RC_Read(T_RC_Data *temp)
{	
//	static uint16_t rc1 = 0,rc2 = 0,rc3 = 0,rc4 = 0;
//	rc1 = Capture1>RC_MAX ? RC_MAX:Capture1;
//	rc1 = Capture1<RC_MIN ? RC_MIN:Capture1;
//	
//	rc2 = Capture2>RC_MAX ? RC_MAX:Capture2;
//	rc2 = Capture2<RC_MIN ? RC_MIN:Capture2;
//	
//	rc3 = Capture3>RC_MAX ? RC_MAX:Capture3;
//	rc3 = Capture3<RC_MIN ? RC_MIN:Capture3;
//	
//	rc4 = Capture4>RC_MAX ? RC_MAX:Capture4;
//	rc4 = Capture4<RC_MIN ? RC_MIN:Capture4;
	
	temp->ROLL		= (int16_t)Capture1;//rc1
	temp->PITCH		= (int16_t)Capture2;//rc2
	temp->THROTTLE	= (int16_t)Capture3;//rc3
	temp->YAW		= (int16_t)Capture4;//rc4
	 
	//Capture5与Capture6是JTAG通道
	temp->AUX1		= (int16_t)Capture7;//rc5;
	temp->AUX2		= (int16_t)Capture8;//rc5;
	temp->AUX3		= 0;
	temp->AUX4		= 0;
	temp->AUX5		= 0;
	temp->AUX6		= 0;
	
	if(temp->AUX1>1520 && temp->AUX1<1750)
		SYSTEM_STA.ALT_HOLD_MODE=1;
	else if(temp->AUX1>1950)
		SYSTEM_STA.ALT_HOLD_MODE=0;
}


