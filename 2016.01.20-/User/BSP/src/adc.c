#include "adc.h"


#define AVE_TIMES 3
#define AD_MAX	4095
#define AD_MIN	0

// PB0��ص�ѹ���
void Adc_cfig(void)
{
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_ADC1,ENABLE );	  //ʹ��ADC1ͨ��ʱ��
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
 			
	/*********PB0 bettary chack************/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	ADC_DeInit(ADC1);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	  //ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;				  //ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	  //ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;								//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);										//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   

	ADC_Cmd(ADC1, ENABLE);																//ʹ��ָ����ADC1	
	ADC_ResetCalibration(ADC1);														//ʹ�ܸ�λУ׼ 
		 
	while(ADC_GetResetCalibrationStatus(ADC1));						//�ȴ���λУ׼����	
	ADC_StartCalibration(ADC1);	 													//����ADУ׼
	while(ADC_GetCalibrationStatus(ADC1));	 							//�ȴ�У׼����
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
  //����ָ��ADC�Ĺ�����ͨ����6�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, channels, ranks, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			     
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}



