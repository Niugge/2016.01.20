#include "adc.h"


#define AVE_TIMES 3
#define AD_MAX	4095
#define AD_MIN	0

// PB0电池电压检测
void Adc_cfig(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_ADC1,ENABLE );	  //使能ADC1通道时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
 			
	/*********PB0 bettary chack************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	ADC_DeInit(ADC1);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	  //ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;				  //模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	  //模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;								//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);										//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

	ADC_Cmd(ADC1, ENABLE);																//使能指定的ADC1	
	ADC_ResetCalibration(ADC1);														//使能复位校准 
		 
	while(ADC_GetResetCalibrationStatus(ADC1));						//等待复位校准结束	
	ADC_StartCalibration(ADC1);	 													//开启AD校准
	while(ADC_GetCalibrationStatus(ADC1));	 							//等待校准结束
}

uint16_t read_adc(uint8_t channel,uint8_t rank)
{
	uint8_t i ,ch ,turn;
	uint16_t sum = 0;
	ch = channel;
	turn = rank;
	for (i=0;i<AVE_TIMES;i++)
		{
			sum += Get_Adc(ch,turn);
//			delay_ms(5);
		}
	return sum/AVE_TIMES;
}

uint16_t Get_Adc(uint8_t channels,uint8_t ranks)   
{
  //设置指定ADC的规则组通道，6个序列，采样时间
	ADC_RegularChannelConfig(ADC1, channels, ranks, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	  			     
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}



