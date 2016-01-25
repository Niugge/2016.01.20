#include "ADNS3080.h"
#include "spi.h"
#include "delay.h"
#include "usart.h"
#include "srom.h"
#include "global.h"

#define H_PIX   4.8//����,  ���    ԭ��200mm
#define H_OBJ  350  //���ף� ���
#define FILTER_NUM 20


//           GPA.11		NPD	,�͵�ƽ˯��,���Բ��ӣ�Ĭ��Ϊ��
//						GPA.12   RESET,��λ��                 ��������������Բ���
//            GPC.3     Ƭѡ
//void ADNS_3080_GPIO_Configuration(void)
//{
////  GPIO_InitTypeDef GPIO_InitStructure;
////  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
////	
////  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
////  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
////  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP; //�������
////  GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_InitTypeDef GPIO_InitStructure;
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
//	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP; //�������
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
//}

void ADNS3080_reset(void)  //ADNS3080 ��λ���ߣ�       ��GPA.12
{
  GPIO_ResetBits(GPIOA,GPIO_Pin_12);                   //
  delay_ms(5);																				//			  ______
  GPIO_SetBits(GPIOA,GPIO_Pin_12);                     //  _____|			|_______����λ�ź�
  delay_ms(5);																			  //
  GPIO_ResetBits(GPIOA,GPIO_Pin_12);//�����ź�
}

void Write_srom(void)                     //�չ̼�
{
 int i;
 ON_CS(); 
 writr_register(0x20,0x44);               //****************************************//
 delay_us(51);                            //
 writr_register(0x23,0x07);               //				�������Ĵ������ֲ���û�У���˵�˾��岽��
  delay_us(51);                           //
 writr_register(0x24,0x88);               //******************************************//
 delay_us(51);
  OFF_CS();  //ͻ��_дģʽ
  delay_us(340);//�ȴ�����1֡ʱ��  ����ÿ��3000֡����340us>1/3000=334us
  ON_CS(); 
  writr_register(SROM_Enable,0x18);
  OFF_CS();  //ͻ��_дģʽ
 delay_us(41);//  >40us
  ON_CS();
  for(i=0;i<=1985;i++)
  {
     writr_register(0x60,SROM[i]);
     delay_us(11);// >10us  ��t_loadʱ��
  }
 OFF_CS();
 delay_us(105);	//>104us
}
void ADNS3080_Init(void)
{	  
  //ADNS_3080_GPIO_Configuration();          //�����ܽŵĳ�ʼ��,��仰��Ҫ�ɲ�Ҫ
  SPI_Simu_Init();  //�ı��ٶȣ�2��256��Ƶ��
  //ADNS3080_reset(); //��λ
  //GPIO_SetBits(GPIOA,GPIO_Pin_11);  //����NPD,��˯��
  delay_ms(10);
  Write_srom();    //��д�̼�
  ADNS_Configuration();
	printf("%d\r\n",read_register(0x1f));	 //�鿴�Ƿ����سɹ�,����ֵ��31���Ƕ���
}

void ADNS_Configuration(void)
{
	 ON_CS(); 
	 writr_register(Configuration_bits,0x10);		//���÷ֱ��� 1600	 //��Bit 4Ϊ0����Ϊ400��ÿӢ��
	 delay_ms(3);
	 writr_register(Extended_Config,0x01);      //����Ϊ�̶�֡�ʣ���ֵ��Frame_Period_Max_Bound�Ĵ�����
	 delay_ms(3);
	 if(read_busy()!=1)
	 {  							      //��Ϊ3000֡ÿ��,3000=24MHz/0x1f40,0x1f40=8000
			OFF_CS();  //ͻ��_дģʽ
			delay_ms(2);
			ON_CS();	
			SPI_Simu_RW(Frame_Period_Max_Bound_Lower+0x80);	//����֡�� //��д��λ��д��λ�����Ƕ��Ļ��ȶ���λ�ٶ���λ
			SPI_Simu_RW(0x40); //   C0�� 5000֡��	   
		 // delay_us(75);
			SPI_Simu_RW(Frame_Period_Max_Bound_Upper+0x80);
			SPI_Simu_RW(0x1f);	 // 12
	 } 
	 clear_motion();
	 OFF_CS();
}

u8 read_register(u8 adress)
{
	u8 temp;
	ON_CS();
	temp=SPI_Simu_RW(adress+0x00);	//��  ,����������ӳٲ��ܶ�����
	delay_us(75);
	temp=SPI_Simu_RW(0xff);	//�ṩʱ���ź�_��
	OFF_CS();
	return temp;
}

void writr_register(u8 adress,u8 vlue)
{
	ON_CS();
	 SPI_Simu_RW(adress+0x80);   //����0x80��ʾд�����λΪ1��д�����λΪ0����
	 SPI_Simu_RW(vlue);
	OFF_CS();
	delay_us(51);
}

u8 read_busy(void)//д֡�ʵ���æ  ==1æ
{
	u8 temp;
	ON_CS();
	temp=SPI_Simu_RW(Extended_Config+0x00);   //8λ�е����λ
	delay_us(75);
	temp=SPI_Simu_RW(0xff);
	temp&=0x80;
	OFF_CS();
	return temp;
}

void clear_motion(void)
{
	ON_CS();
	 SPI_Simu_RW(Motion_Clear+0x80);
	 SPI_Simu_RW(0xff);	//���X Y����
	OFF_CS();
}


u16 read_zhenlv(void) //��֡��
{  
  u16 Frame_Period_Max_Bound_Lower1,Frame_Period_Max_Bound_Upper1;
  ON_CS();
  Frame_Period_Max_Bound_Upper1=SPI_Simu_RW(Frame_Period_Uppe+0x00);
	delay_us(51);        //�����ʱ����Ҫ��������������ݲ���ȷ
  Frame_Period_Max_Bound_Upper1=SPI_Simu_RW(0xff);//���ո�λ��֡��
  //delay_ms(5);      //���ʱ���Ҫ�ɲ�Ҫ
  Frame_Period_Max_Bound_Lower1=SPI_Simu_RW(Frame_Period_Lower+0x00);
	delay_us(51);      //�����ʱ����Ҫ��������������ݲ���ȷ
  Frame_Period_Max_Bound_Lower1=SPI_Simu_RW(0xff); //���յ�λ��֡��
  OFF_CS();
  return ((Frame_Period_Max_Bound_Upper1 << 8) | Frame_Period_Max_Bound_Lower1);
}

void Read_Data_burst(void)       //��ȡƽ��λ��
{
  static int SumX;
  static int SumY;
  int sum_x,sum_y;
  unsigned char move=0;
  int  x=0;              //��ʾ�����һ�ζ�λ��Ϣ���ʱ�̵�λ������
  int  y=0;
//burst����������������������������������������������������������������������������
	
	//Spi_Change_To_ADNS3080_();
	
	ON_CS();
	SPI_Simu_RW(Motion_Burst);   //������ʽ��
	delay_us(75);
	move=SPI_Simu_RW(0xFF);             
	x=SPI_Simu_RW(0xFF);
	y=SPI_Simu_RW(0xFF);
	if(x&0x80)
	  {
	  //x�Ķ�����ת��	
	  x -= 1;                  //*******************************************************//
	  x = ~x;									 //
	  x=(-1)*x;                // �����ע�⣬��ȡ��detal_x,detal_y��8λ�з��ŵģ��������x,y��16λ�����з���
	  x-=256;                  //���ԣ�ת����ע��
 	  }                        //
	if(y&0x80)                 //********************************************************//
	  {
	  //y�Ķ�����ת��	
	  y -= 1;
	  y = ~y;	
	  y=(-1)*y;
	  y-=256;
	  } 
	SumX=SumX+x;             //�ۼ�X������ƶ�����
	SumY=SumY+y;			 //�ۼ�Y������ƶ�����
	OFF_CS();
	delay_us(4);  //������ʱ4um���˳�motion modeģʽ
	OFF_CS();
 	sum_x=(25.4*(float)SumX *H_OBJ)/(H_PIX*1600);//����=d_x*(25.4/1600)*n   ���г�����n=���:���=8����:�ﳤ
  sum_y=(25.4*(float)SumY *H_OBJ)/(H_PIX*1600);
  if((move&0x10)!=1)   //��û�����
	{
		if(move&0x80)      //�������ݿ��Զ�����
		{
			Data_Filter(sum_x,sum_y);//�����˲�
			printf("%d,%d\n",sum_filter_x,sum_filter_y);   //x,y
		}
	}
	else
	{
		x=0;
		y=0;
	}
x=0;
y=0;
	
	//Spi_Change_To_nRF24L01_();
	
}

float  read_average_pixel(void)	  //��ƽ������
{
  float temp;
  ON_CS();
  temp=SPI_Simu_RW(Pixel_Sum);
  delay_us(76);
  temp=SPI_Simu_RW(0xff);         //�˴�������ֵ���Ϊ221
  temp=temp*256/900;                  //temp���Ϊ63
  OFF_CS();
  return temp;
}

u8  read_Maximum_pixel(void)	  //���������
{
  u8 temp;
  ON_CS();
  temp=SPI_Simu_RW(Maximum_Pixel);
  delay_us(76);
  temp=SPI_Simu_RW(0xff);       //temp���Ϊ63              
  OFF_CS();
  return temp;
}

void read_pixel(void)           //������
{
    u8 i,j ,regValue, pixelValue,test=1;	 
		writr_register(Frame_Capture,0x83); //�Ĵ�����д��0x83����֡��׽
		delay_us(1010);//�ȴ�3֡+10us=  (1/3000)*1000000*3+10 =1010us
  //��ʾ����  30*30=900
  for(i=0;i<30;i++)//��
  {
	  for(j=0;j<30;j++) //�� 
	  {
	   regValue=read_register(Frame_Capture);  //������
	    if( test && ((regValue&0x40)==0)) //�Ҳ�����һ�����أ���һ֡��һ�����ص�bit6λ1���������
			{
					printf("Read pixel fail");		    
			}
			test=0;
			pixelValue =(regValue<<2);
			while(!(USART1->SR&(1<<6)));
				USART1->DR=pixelValue;                               
									delay_us(50);
			}
   }
  ADNS3080_reset();//��������
}

void read_pixel_burst(void)//������ͼ��
{
	int i,j;
	writr_register(Frame_Capture,0x83); 
	delay_us(1010);//�ȴ�3֡ (1/3000)*1000000*3+10 =1010us
	//��ʼburst��
	ON_CS();
	SPI_Simu_RW(0x40);   //0x40ΪPixel_Burst	�Ĵ���
	delay_us(75);
	for(i=0;i<30;i++)
	{
		for(j=0;j<30;j++)
		{
		while(!(USART1->SR&(1<<6)));
		USART1->DR=(SPI_Simu_RW(0xFF)<<2); 
			//delay_us(10);
		}
	} 
	OFF_CS();
	delay_us(4);
	OFF_CS();            
}

void Data_Filter(int detal_x,int detal_y)//�����˲�
{
	static uint8_t 	filter_cnt=0;
	static int16_t	Detal_X_BUF[FILTER_NUM],Detal_Y_BUF[FILTER_NUM];
	int32_t temp1=0,temp2=0;
	uint8_t i;

	Detal_X_BUF[filter_cnt] = detal_x;
	Detal_Y_BUF[filter_cnt] = detal_y;

	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += Detal_X_BUF[i];
		temp2 += Detal_Y_BUF[i];
	}

	sum_filter_x = (vs32)(temp1 / FILTER_NUM);
	sum_filter_y = (vs32)(temp2 / FILTER_NUM);

	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;
	
	Gps_Data.LAT=sum_filter_x;
	Gps_Data.LNG=sum_filter_y;
}
