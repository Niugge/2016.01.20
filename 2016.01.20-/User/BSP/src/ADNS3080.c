#include "ADNS3080.h"
#include "spi.h"
#include "delay.h"
#include "usart.h"
#include "srom.h"
#include "global.h"

#define H_PIX   4.8//毫米,  像高    原来200mm
#define H_OBJ  350  //毫米， 物高
#define FILTER_NUM 20


//           GPA.11		NPD	,低电平睡眠,可以不接，默认为高
//						GPA.12   RESET,复位脚                 ，推挽输出，可以不接
//            GPC.3     片选
//void ADNS_3080_GPIO_Configuration(void)
//{
////  GPIO_InitTypeDef GPIO_InitStructure;
////  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
////	
////  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
////  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
////  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP; //推挽输出
////  GPIO_Init(GPIOA, &GPIO_InitStructure);
//	GPIO_InitTypeDef GPIO_InitStructure;
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
//	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode =GPIO_Mode_Out_PP; //推挽输出
//  GPIO_Init(GPIOC, &GPIO_InitStructure);
//}

void ADNS3080_reset(void)  //ADNS3080 复位（高）       ，GPA.12
{
  GPIO_ResetBits(GPIOA,GPIO_Pin_12);                   //
  delay_ms(5);																				//			  ______
  GPIO_SetBits(GPIOA,GPIO_Pin_12);                     //  _____|			|_______，复位信号
  delay_ms(5);																			  //
  GPIO_ResetBits(GPIOA,GPIO_Pin_12);//脉冲信号
}

void Write_srom(void)                     //烧固件
{
 int i;
 ON_CS(); 
 writr_register(0x20,0x44);               //****************************************//
 delay_us(51);                            //
 writr_register(0x23,0x07);               //				这三个寄存器，手册上没有，但说了具体步骤
  delay_us(51);                           //
 writr_register(0x24,0x88);               //******************************************//
 delay_us(51);
  OFF_CS();  //突发_写模式
  delay_us(340);//等待大于1帧时间  ？？每秒3000帧，则340us>1/3000=334us
  ON_CS(); 
  writr_register(SROM_Enable,0x18);
  OFF_CS();  //突发_写模式
 delay_us(41);//  >40us
  ON_CS();
  for(i=0;i<=1985;i++)
  {
     writr_register(0x60,SROM[i]);
     delay_us(11);// >10us  ，t_load时间
  }
 OFF_CS();
 delay_us(105);	//>104us
}
void ADNS3080_Init(void)
{	  
  //ADNS_3080_GPIO_Configuration();          //两个管脚的初始化,这句话可要可不要
  SPI_Simu_Init();  //改变速度（2到256分频）
  //ADNS3080_reset(); //复位
  //GPIO_SetBits(GPIOA,GPIO_Pin_11);  //拉高NPD,免睡眠
  delay_ms(10);
  Write_srom();    //烧写固件
  ADNS_Configuration();
	printf("%d\r\n",read_register(0x1f));	 //查看是否下载成功,返回值是31就是对了
}

void ADNS_Configuration(void)
{
	 ON_CS(); 
	 writr_register(Configuration_bits,0x10);		//设置分辨率 1600	 //若Bit 4为0，则为400点每英寸
	 delay_ms(3);
	 writr_register(Extended_Config,0x01);      //设置为固定帧率，其值在Frame_Period_Max_Bound寄存器中
	 delay_ms(3);
	 if(read_busy()!=1)
	 {  							      //设为3000帧每秒,3000=24MHz/0x1f40,0x1f40=8000
			OFF_CS();  //突发_写模式
			delay_ms(2);
			ON_CS();	
			SPI_Simu_RW(Frame_Period_Max_Bound_Lower+0x80);	//设置帧率 //先写低位再写高位，若是读的话先读高位再读低位
			SPI_Simu_RW(0x40); //   C0， 5000帧率	   
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
	temp=SPI_Simu_RW(adress+0x00);	//读  ,后面必须有延迟才能读数据
	delay_us(75);
	temp=SPI_Simu_RW(0xff);	//提供时钟信号_读
	OFF_CS();
	return temp;
}

void writr_register(u8 adress,u8 vlue)
{
	ON_CS();
	 SPI_Simu_RW(adress+0x80);   //加上0x80表示写，最高位为1：写，最高位为0：读
	 SPI_Simu_RW(vlue);
	OFF_CS();
	delay_us(51);
}

u8 read_busy(void)//写帧率的判忙  ==1忙
{
	u8 temp;
	ON_CS();
	temp=SPI_Simu_RW(Extended_Config+0x00);   //8位中的最高位
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
	 SPI_Simu_RW(0xff);	//清除X Y数据
	OFF_CS();
}


u16 read_zhenlv(void) //读帧率
{  
  u16 Frame_Period_Max_Bound_Lower1,Frame_Period_Max_Bound_Upper1;
  ON_CS();
  Frame_Period_Max_Bound_Upper1=SPI_Simu_RW(Frame_Period_Uppe+0x00);
	delay_us(51);        //这个延时很重要，否则读出的数据不正确
  Frame_Period_Max_Bound_Upper1=SPI_Simu_RW(0xff);//接收高位的帧率
  //delay_ms(5);      //这个时间可要可不要
  Frame_Period_Max_Bound_Lower1=SPI_Simu_RW(Frame_Period_Lower+0x00);
	delay_us(51);      //这个延时很重要，否则读出的数据不正确
  Frame_Period_Max_Bound_Lower1=SPI_Simu_RW(0xff); //接收低位的帧率
  OFF_CS();
  return ((Frame_Period_Max_Bound_Upper1 << 8) | Frame_Period_Max_Bound_Lower1);
}

void Read_Data_burst(void)       //读取平面位移
{
  static int SumX;
  static int SumY;
  int sum_x,sum_y;
  unsigned char move=0;
  int  x=0;              //表示相对上一次定位信息输出时刻的位移增量
  int  y=0;
//burst读。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。
	
	//Spi_Change_To_ADNS3080_();
	
	ON_CS();
	SPI_Simu_RW(Motion_Burst);   //爆发方式读
	delay_us(75);
	move=SPI_Simu_RW(0xFF);             
	x=SPI_Simu_RW(0xFF);
	y=SPI_Simu_RW(0xFF);
	if(x&0x80)
	  {
	  //x的二补码转换	
	  x -= 1;                  //*******************************************************//
	  x = ~x;									 //
	  x=(-1)*x;                // 这里得注意，读取的detal_x,detal_y是8位有符号的，而定义的x,y是16位整型有符号
	  x-=256;                  //所以，转换得注意
 	  }                        //
	if(y&0x80)                 //********************************************************//
	  {
	  //y的二补码转换	
	  y -= 1;
	  y = ~y;	
	  y=(-1)*y;
	  y-=256;
	  } 
	SumX=SumX+x;             //累加X读入的移动数据
	SumY=SumY+y;			 //累加Y读入的移动数据
	OFF_CS();
	delay_us(4);  //必须延时4um来退出motion mode模式
	OFF_CS();
 	sum_x=(25.4*(float)SumX *H_OBJ)/(H_PIX*1600);//距离=d_x*(25.4/1600)*n   其中成像倍率n=像高:物高=8毫米:物长
  sum_y=(25.4*(float)SumY *H_OBJ)/(H_PIX*1600);
  if((move&0x10)!=1)   //若没有溢出
	{
		if(move&0x80)      //若有数据可以读出来
		{
			Data_Filter(sum_x,sum_y);//滑动滤波
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

float  read_average_pixel(void)	  //读平均像素
{
  float temp;
  ON_CS();
  temp=SPI_Simu_RW(Pixel_Sum);
  delay_us(76);
  temp=SPI_Simu_RW(0xff);         //此处读出的值最大为221
  temp=temp*256/900;                  //temp最大为63
  OFF_CS();
  return temp;
}

u8  read_Maximum_pixel(void)	  //读最大像素
{
  u8 temp;
  ON_CS();
  temp=SPI_Simu_RW(Maximum_Pixel);
  delay_us(76);
  temp=SPI_Simu_RW(0xff);       //temp最大为63              
  OFF_CS();
  return temp;
}

void read_pixel(void)           //读像素
{
    u8 i,j ,regValue, pixelValue,test=1;	 
		writr_register(Frame_Capture,0x83); //寄存器中写入0x83开启帧捕捉
		delay_us(1010);//等待3帧+10us=  (1/3000)*1000000*3+10 =1010us
  //显示数据  30*30=900
  for(i=0;i<30;i++)//列
  {
	  for(j=0;j<30;j++) //行 
	  {
	   regValue=read_register(Frame_Capture);  //读像素
	    if( test && ((regValue&0x40)==0)) //找不到第一个像素，第一帧第一个像素的bit6位1则读到数据
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
  ADNS3080_reset();//重启运行
}

void read_pixel_burst(void)//爆发读图像
{
	int i,j;
	writr_register(Frame_Capture,0x83); 
	delay_us(1010);//等待3帧 (1/3000)*1000000*3+10 =1010us
	//开始burst读
	ON_CS();
	SPI_Simu_RW(0x40);   //0x40为Pixel_Burst	寄存器
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

void Data_Filter(int detal_x,int detal_y)//滑动滤波
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
