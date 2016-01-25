#include "key.h"

//������ʼ������ 
//PA15��PC5 ���ó�����
void KEY_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;

// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC,ENABLE);//ʹ��PORTA,PORTCʱ��

//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//�ر�jtag��ʹ��SWD��������SWDģʽ����
//	
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_15;//PA15
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
// 	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA15
//	 
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0;//PA0
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0���ó����룬Ĭ������	  
//	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.0
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//ʹ��PORTCʱ��
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1 | GPIO_Pin_13;//PA15
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
 	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIOc
} 
//����������
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//����ֵ��
//0��û���κΰ�������
//KEY1_PRES��KEY1����
//WKUP_PRES��WK_UP���� 
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>WK_UP!!
uint8_t KEY_Scan(uint8_t mode)
{	 
//	static uint8_t key_up=1;//�������ɿ���־
//	if(mode)key_up=1;  //֧������		  
//	if(key_up&&(KEY1==0||WK_UP==1))
//	{
//		delay_ms(10);		 //ȥ���� 
//		key_up=0;
//		if(KEY1==0)return KEY1_PRES;
//		else if(WK_UP==1)return WKUP_PRES; 
//	}else if(KEY1==1&&WK_UP==0)key_up=1; 	     
//	return 0;          // �ް�������
	if(KEY0==0)
	{
		delay_ms(15);
		if(KEY0==0)
		{
			while(!KEY0);
			return 1;
		}
	}
	
	if(KEY1==0)
	{
		delay_ms(15);
		if(KEY1==0)
		{
			while(!KEY1);
			return 2;
		}
	}
	return 0;
}

void  Calibration_Sensor(void)
{
	uint8_t t=0;	 	
	t=KEY_Scan(0);		//�õ���ֵ
//	if(t>0)
//	{
//		switch(t)
//		{				 
//			/******����������ѹ��******/
//			case KEY1_PRES:
//				MPU6050_CalOff_Gyr();
//				MS5611_CalOffset();	
//				MPU6050_CalOff_Acc();
//				break;			
//			/*********��ǿ��***********/
//			case WKUP_PRES:	
//				Cal_Compass();			
//				break;	
//			
//			default:
//				break;	
//		}	
//	
//	}

		switch(t)
		{				 
			/******����������ѹ��******/
			case KEY1_PRES:
				MPU6050_CalOff_Gyr();
				//MS5611_CalOffset();	
				MPU6050_CalOff_Acc();
				break;			
			/*********��ǿ��***********/
			case KEY2_PRES:	
				//Cal_Compass();	
				MS5611_CalOffset();				
				break;	
			
			default:
				break;	
		}	

}

