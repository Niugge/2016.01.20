#include "DATA_Transfer.h"


//将16位数据分成2个8位数据发
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))	

uint8_t Send_Offset = 0;
/********读上位机下发数据**********/
void Nrf_Check_Event(void)
{
	uint8_t sta = NRF_Read_Reg(NRF_READ_REG + STATUS);
	if(sta & (1<<RX_DR))
	{
		uint8_t rx_len = NRF_Read_Reg(R_RX_PL_WID);
		if(rx_len<33)
		{
			NRF_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,rx_len);// read receive payload from RX_FIFO buffer
			/**********DATA analyze************/
			Data_Receive_Anl(NRF24L01_RXDATA,rx_len,&Rc_D,&PID1_ipt,&PID2_ipt);
		}
		else if(rx_len == 0)
		{
			NRF_Write_Reg(FLUSH_RX,0xff);//清空缓冲区			
		}
	}

	if(sta & (1<<TX_DS))
	{

	}

	if(sta & (1<<MAX_RT))
	{
		if(sta & 0x01)	
		{
			NRF_Write_Reg(FLUSH_TX,0xff);
		}
	}
	NRF_Write_Reg(NRF_WRITE_REG+STATUS,sta);
	sta = Nrf_Get_FIFOSta();
	NRF_Write_Reg(FLUSH_RX,0xff);//清空缓冲区

}

void Data_Receive_Anl(uint8_t *data_buf,uint8_t num,T_RC_Data *rc_in,PID *pid1_in,PID_RATE *pid2_in)
{
	uint8_t sum = 0,i=0;
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
			{MPU6050_CalOff_Acc();red_toggle();}

		else if(*(data_buf+4)==0X02)
			{MPU6050_CalOff_Gyr();red_toggle();}
		
		else if(*(data_buf+4)==0X03)
			{MPU6050_CalOff_Acc();MPU6050_CalOff_Gyr();	red_toggle();}
			
		else if(*(data_buf+4)==0X04)
			{Cal_Compass();red_toggle();}
		
		else if(*(data_buf+4)==0X05)
		{MS5611_CalOffset(); red_toggle();}
	}
	
	else if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			Send_PID1 = 1;
			red_toggle();
		}
//		if(*(data_buf+4)==0X02)
//		{
//			Send_Offset = 1;
//		}
	}
#ifdef CONTROL_USE_24L01
	else if(*(data_buf+2)==0X03)
	{
		static uint8_t i = 0;
		i++;
		if(i == 20)
			{ red_toggle(); i = 0;}		
		/********根据凤凰模拟器修改可以保证5个通道**************/
		rc_in->AUX1 =    (vs16)(*(data_buf+4)<<8)|*(data_buf+5);
		rc_in->ROLL =    (vs16)(*(data_buf+6)<<8)|*(data_buf+7);
		rc_in->ROLL = 	  3000 - rc_in->ROLL;
		rc_in->YAW  =    (vs16)(*(data_buf+8)<<8)|*(data_buf+9);
		rc_in->YAW  =     3000 - rc_in->YAW;
		rc_in->THROTTLE =(vs16)(*(data_buf+10)<<8)|*(data_buf+11);
		rc_in->THROTTLE = 3000 - rc_in->THROTTLE;//油门反向
		rc_in->PITCH= 	 (vs16)(*(data_buf+12)<<8)|*(data_buf+13);
		rc_in->PITCH=     3000 - rc_in->PITCH;
		rc_in->AUX2 = 	 (vs16)(*(data_buf+14)<<8)|*(data_buf+15);
		rc_in->AUX3 = 	 (vs16)(*(data_buf+16)<<8)|*(data_buf+17);
		rc_in->AUX4 = 	 (vs16)(*(data_buf+18)<<8)|*(data_buf+19);
		rc_in->AUX5 = 	 (vs16)(*(data_buf+20)<<8)|*(data_buf+21);
		rc_in->AUX6 = 	 (vs16)(*(data_buf+21)<<8)|*(data_buf+22);				
	}
#endif
	else if(*(data_buf+2)==0X10)								//PID1
	{
		red_toggle();
		pid1_in->ROL_P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
		pid1_in->ROL_I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
		pid1_in->ROL_D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
		pid1_in->PIT_P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
		pid1_in->PIT_I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
		pid1_in->PIT_D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
		pid1_in->YAW_P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
		pid1_in->YAW_I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/1000;
		pid1_in->YAW_D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
		Data_Send_Check(sum);
		EE_SAVE_PID1();
	}
	else if(*(data_buf+2)==0X11)								//PID2
	{
		pid1_in->ALT_P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
		pid1_in->ALT_I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
		pid1_in->ALT_D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
		pid1_in->POS_P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
		pid1_in->POS_I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
		pid1_in->POS_D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
		pid1_in->PID1_P= (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
		pid1_in->PID1_I= (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/10000;//数值很小，缩放很大
		pid1_in->PID1_D= (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
		Data_Send_Check(sum);
		EE_SAVE_PID2();
	}
	else if(*(data_buf+2)==0X12)								//PID3
	{
			pid2_in->ROL_RATE_P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
			pid2_in->ROL_RATE_I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
			pid2_in->ROL_RATE_D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
			pid2_in->PIT_RATE_P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;
			pid2_in->PIT_RATE_I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
			pid2_in->PIT_RATE_D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			pid2_in->YAW_RATE_P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;
			pid2_in->YAW_RATE_I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/1000;
			pid2_in->YAW_RATE_D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
			EE_SAVE_PID3();
	}
	else if(*(data_buf+2)==0X13)								//PID4
	{
			pid2_in->ALT_RATE_P = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/100;
			pid2_in->ALT_RATE_I = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
			pid2_in->ALT_RATE_D = (float)((vs16)(*(data_buf+8)<<8)|*(data_buf+9))/100;
			pid2_in->POS_RATE_P = (float)((vs16)(*(data_buf+10)<<8)|*(data_buf+11))/100;  //
			pid2_in->POS_RATE_I = (float)((vs16)(*(data_buf+12)<<8)|*(data_buf+13))/1000;
			pid2_in->POS_RATE_D = (float)((vs16)(*(data_buf+14)<<8)|*(data_buf+15))/100;
			pid2_in->PID1_RATE_P = (float)((vs16)(*(data_buf+16)<<8)|*(data_buf+17))/100;//
			pid2_in->PID1_RATE_I = (float)((vs16)(*(data_buf+18)<<8)|*(data_buf+19))/1000;
			pid2_in->PID1_RATE_D = (float)((vs16)(*(data_buf+20)<<8)|*(data_buf+21))/100;
			Data_Send_Check(sum);
			EE_SAVE_PID4();
	}
//	else if(*(data_buf+2)==0X16)								//OFFSET
//	{
//			AngleOffset_Rol = (float)((vs16)(*(data_buf+4)<<8)|*(data_buf+5))/1000;
//			AngleOffset_Pit = (float)((vs16)(*(data_buf+6)<<8)|*(data_buf+7))/1000;
//	}
}
/*********将需要发送的数据打包**********/
void Data_Send_RCData(T_RC_Data *rc_opt)
{
	uint8_t _cnt=0;
	uint8_t sum = 0 ,i = 0;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0x03;
	NRF24L01_TXDATA[_cnt++]=0;			
	/**********飞控数据***********/
	/*4:5 thresth*/
	NRF24L01_TXDATA[_cnt++]=BYTE1(rc_opt->THROTTLE);
	NRF24L01_TXDATA[_cnt++]=BYTE0(rc_opt->THROTTLE);
	/*6:7 yaw */
	NRF24L01_TXDATA[_cnt++]=BYTE1(rc_opt->YAW);
	NRF24L01_TXDATA[_cnt++]=BYTE0(rc_opt->YAW);
	/*8:9 roll*/
	NRF24L01_TXDATA[_cnt++]=BYTE1(rc_opt->ROLL);
	NRF24L01_TXDATA[_cnt++]=BYTE0(rc_opt->ROLL);
	/*10:11 pitch*/
	NRF24L01_TXDATA[_cnt++]=BYTE1(rc_opt->PITCH);
	NRF24L01_TXDATA[_cnt++]=BYTE0(rc_opt->PITCH);
	
	NRF24L01_TXDATA[_cnt++]=BYTE1(rc_opt->AUX1);//
	NRF24L01_TXDATA[_cnt++]=BYTE0(rc_opt->AUX1);//
	NRF24L01_TXDATA[_cnt++]=BYTE1(rc_opt->AUX2);//
	NRF24L01_TXDATA[_cnt++]=BYTE0(rc_opt->AUX2);//
	NRF24L01_TXDATA[_cnt++]=BYTE1(rc_opt->AUX3);//
	NRF24L01_TXDATA[_cnt++]=BYTE0(rc_opt->AUX3);//
	NRF24L01_TXDATA[_cnt++]=BYTE1(rc_opt->AUX4);//
	NRF24L01_TXDATA[_cnt++]=BYTE0(rc_opt->AUX4);//
	NRF24L01_TXDATA[_cnt++]=BYTE1(rc_opt->AUX5);//
	NRF24L01_TXDATA[_cnt++]=BYTE0(rc_opt->AUX5);//
	NRF24L01_TXDATA[_cnt++]=BYTE1(rc_opt->AUX6);//
	NRF24L01_TXDATA[_cnt++]=BYTE0(rc_opt->AUX6);//
	
	/***数据总数发往飞控，校验位***/
	NRF24L01_TXDATA[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[_cnt++]=sum;
	/********启动2401开始发数据**********/
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(NRF24L01_TXDATA,_cnt);
#else
	NRF_TxPacket(NRF24L01_TXDATA,_cnt);
#endif
}

//                  经纬度
void Data_Send_GPSData(GPS_Data *GPS_opt)
{
	uint8_t _cnt=0;
	uint8_t sum = 0 ,i = 0;
	vs32 _temp = 0;
	
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0x04;
	NRF24L01_TXDATA[_cnt++]=0;			
	/**********飞控数据***********/
	/*4:5:6:7 LAT*/
	NRF24L01_TXDATA[_cnt++]=BYTE3(GPS_opt->LAT);
	NRF24L01_TXDATA[_cnt++]=BYTE2(GPS_opt->LAT);
	NRF24L01_TXDATA[_cnt++]=BYTE1(GPS_opt->LAT);
	NRF24L01_TXDATA[_cnt++]=BYTE0(GPS_opt->LAT);
	/*8:9:10:11 LNG */
	NRF24L01_TXDATA[_cnt++]=BYTE3(GPS_opt->LNG);
	NRF24L01_TXDATA[_cnt++]=BYTE2(GPS_opt->LNG);
	NRF24L01_TXDATA[_cnt++]=BYTE1(GPS_opt->LNG);
	NRF24L01_TXDATA[_cnt++]=BYTE0(GPS_opt->LNG);
	/*12:13:14:15 */
	NRF24L01_TXDATA[_cnt++]=BYTE3(GPS_opt->GPS_ALT);
	NRF24L01_TXDATA[_cnt++]=BYTE2(GPS_opt->GPS_ALT);
	NRF24L01_TXDATA[_cnt++]=BYTE1(GPS_opt->GPS_ALT);
	NRF24L01_TXDATA[_cnt++]=BYTE0(GPS_opt->GPS_ALT);
	/*16:17:18:19 pitch*/
	NRF24L01_TXDATA[_cnt++]=BYTE3(GPS_opt->GPS_SPD);
	NRF24L01_TXDATA[_cnt++]=BYTE2(GPS_opt->GPS_SPD);
	NRF24L01_TXDATA[_cnt++]=BYTE1(GPS_opt->GPS_SPD);
	NRF24L01_TXDATA[_cnt++]=BYTE0(GPS_opt->GPS_SPD);
	
	NRF24L01_TXDATA[_cnt++]=BYTE1(GPS_opt->GPS_HAC);//
	NRF24L01_TXDATA[_cnt++]=BYTE0(GPS_opt->GPS_HAC);//
	
	NRF24L01_TXDATA[_cnt++]=BYTE1(GPS_opt->GPS_VAC);//
	NRF24L01_TXDATA[_cnt++]=BYTE0(GPS_opt->GPS_VAC);//
	
	NRF24L01_TXDATA[_cnt++]=GPS_opt->GPS_STA;//
	NRF24L01_TXDATA[_cnt++]=GPS_opt->GPS_SVN;//

	/***数据总数发往飞控，校验位***/
	NRF24L01_TXDATA[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[_cnt++]=sum;
	/********启动2401开始发数据**********/
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(NRF24L01_TXDATA,_cnt);
#else
	NRF_TxPacket(NRF24L01_TXDATA,_cnt);
#endif
}

void Data_Send_IMU_Data(MPU6050_DATA *pt1,HMC5883L_DATA *pt2)
{
	uint8_t _cnt=0;
	uint8_t sum = 0;
	uint8_t i=0;
	vs16 _temp;
	//帧头
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0x02;
	NRF24L01_TXDATA[_cnt++]=0;

	NRF24L01_TXDATA[_cnt++]=BYTE1(pt1->ax);
	NRF24L01_TXDATA[_cnt++]=BYTE0(pt1->ax);
	NRF24L01_TXDATA[_cnt++]=BYTE1(pt1->ay);
	NRF24L01_TXDATA[_cnt++]=BYTE0(pt1->ay);	
	NRF24L01_TXDATA[_cnt++]=BYTE1(pt1->az);
	NRF24L01_TXDATA[_cnt++]=BYTE0(pt1->az);
	

	NRF24L01_TXDATA[_cnt++]=BYTE1(pt1->gx);
	NRF24L01_TXDATA[_cnt++]=BYTE0(pt1->gx);
	NRF24L01_TXDATA[_cnt++]=BYTE1(pt1->gy);
	NRF24L01_TXDATA[_cnt++]=BYTE0(pt1->gy);
	NRF24L01_TXDATA[_cnt++]=BYTE1(pt1->gz);
	NRF24L01_TXDATA[_cnt++]=BYTE0(pt1->gz);
	
	NRF24L01_TXDATA[_cnt++]=BYTE1(pt2->hx);
	NRF24L01_TXDATA[_cnt++]=BYTE0(pt2->hx);
	NRF24L01_TXDATA[_cnt++]=BYTE1(pt2->hy);
	NRF24L01_TXDATA[_cnt++]=BYTE0(pt2->hy);
	NRF24L01_TXDATA[_cnt++]=BYTE1(pt2->hz);
	NRF24L01_TXDATA[_cnt++]=BYTE0(pt2->hz);
	/*校验部分*/
	NRF24L01_TXDATA[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[_cnt++] = sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(NRF24L01_TXDATA,_cnt);
#else
	NRF_TxPacket(NRF24L01_TXDATA,_cnt);
#endif

}

void Data_Send_MotoPWM(MOTO_PWM *moto_opt)
{
	uint8_t _cnt=0;
	uint8_t i=0,sum = 0;
	uint16_t moto1,moto2,moto3,moto4;
	moto1 = (uint16_t)(moto_opt->MOTO1_PWM - 1000);
	moto2 = (uint16_t)(moto_opt->MOTO2_PWM - 1000);
	moto3 = (uint16_t)(moto_opt->MOTO3_PWM - 1000);
	moto4 = (uint16_t)(moto_opt->MOTO4_PWM - 1000);
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0x06;
	NRF24L01_TXDATA[_cnt++]=0;
	NRF24L01_TXDATA[_cnt++]=BYTE1(moto1);
	NRF24L01_TXDATA[_cnt++]=BYTE0(moto1);
	
	NRF24L01_TXDATA[_cnt++]=BYTE1(moto2);
	NRF24L01_TXDATA[_cnt++]=BYTE0(moto2);
	
	NRF24L01_TXDATA[_cnt++]=BYTE1(moto3);
	NRF24L01_TXDATA[_cnt++]=BYTE0(moto3);
	
	NRF24L01_TXDATA[_cnt++]=BYTE1(moto4);
	NRF24L01_TXDATA[_cnt++]=BYTE0(moto4);
	
	NRF24L01_TXDATA[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(NRF24L01_TXDATA,_cnt);
#else
	NRF_TxPacket(NRF24L01_TXDATA,_cnt);
#endif
}

void Data_Send_Status(ANGEL *angel,vs32 *baro_alt,uint8_t armed)
{
	uint8_t _cnt=0;
	uint8_t sum = 0;
	uint8_t i=0;
	vs16 _temp;
	vs32 _temp2 = 0;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0x01;
	NRF24L01_TXDATA[_cnt++]=0;

	_temp = (int16_t)(-angel->roll*100);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	_temp = (int16_t)(-angel->pitch*100);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	_temp = (int16_t)(-angel->yaw*100);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	NRF24L01_TXDATA[_cnt++] = 0;
	NRF24L01_TXDATA[_cnt++] = 0;	
	
	_temp2 = *baro_alt;
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp2);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp2);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp2);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp2);
		
	if(armed==0)			NRF24L01_TXDATA[_cnt++]=0xA0;	//锁定
	else if(armed==1)		NRF24L01_TXDATA[_cnt++]=0xA1;
	
	NRF24L01_TXDATA[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += NRF24L01_TXDATA[i];
	NRF24L01_TXDATA[_cnt++]=sum;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(NRF24L01_TXDATA,_cnt);
#else
	NRF_TxPacket(NRF24L01_TXDATA,_cnt);
#endif
}

void Data_Send_PID1(PID *pid1)
{
	uint8_t _cnt=0;
	u8 sum = 0;
	uint8_t i=0;
	vs16 _temp;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0x10;
	NRF24L01_TXDATA[_cnt++]=0;
	
	_temp = (int16_t)(pid1->ROL_P * 100);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)(pid1->ROL_I * 1000);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)(pid1->ROL_D * 100);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	_temp = (int16_t)(pid1->PIT_P * 100);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)(pid1->PIT_I * 1000);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)(pid1->PIT_D * 100);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	_temp = (int16_t)(pid1->YAW_P * 100);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)(pid1->YAW_I * 1000);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = (int16_t)(pid1->YAW_D * 100);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	NRF24L01_TXDATA[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(NRF24L01_TXDATA,_cnt);
#else
	NRF_TxPacket(NRF24L01_TXDATA,_cnt);
#endif
}
void Data_Send_PID2(PID *pid1)
{
	uint8_t _cnt=0;
	uint8_t sum = 0;
	uint8_t i=0;
	vs16 _temp;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0x11;
	NRF24L01_TXDATA[_cnt++]=0;
	
	_temp = (pid1->ALT_P * 100);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = (pid1->ALT_I * 1000);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = (pid1->ALT_D * 100);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	_temp = pid1->POS_P * 100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid1->POS_I * 1000;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid1->POS_D * 100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	_temp = pid1->PID1_P * 100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid1->PID1_I * 10000;//数值很小，缩放很大
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid1->PID1_D * 100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	NRF24L01_TXDATA[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(NRF24L01_TXDATA,_cnt);
#else
	NRF_TxPacket(NRF24L01_TXDATA,_cnt);
#endif
}

void Data_Send_PID3(PID_RATE *pid2)
{
	uint8_t _cnt=0;
	uint8_t sum = 0;
	uint8_t i=0;
	vs16 _temp;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0x12;
	NRF24L01_TXDATA[_cnt++]=0;
	
	_temp = pid2->ROL_RATE_P * 100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid2->ROL_RATE_I * 1000;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid2->ROL_RATE_D * 100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	_temp = pid2->PIT_RATE_P * 100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid2->PIT_RATE_I * 1000;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid2->PIT_RATE_D * 100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	_temp = pid2->YAW_RATE_P * 100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid2->YAW_RATE_I * 1000;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid2->YAW_RATE_D * 100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	NRF24L01_TXDATA[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(NRF24L01_TXDATA,_cnt);
#else
	NRF_TxPacket(NRF24L01_TXDATA,_cnt);
#endif
}

void Data_Send_PID4(PID_RATE *pid2)
{
	uint8_t _cnt=0;
	uint8_t sum = 0;
	uint8_t i=0;
	vs16 _temp;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0x13;
	NRF24L01_TXDATA[_cnt++]=0;
	
	_temp = pid2->ALT_RATE_P * 100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid2->ALT_RATE_I * 1000;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid2->ALT_RATE_D * 100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	_temp = pid2->POS_RATE_P * 100;//
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid2->POS_RATE_I * 1000;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid2->POS_RATE_D * 100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	_temp = pid2->PID1_RATE_P * 100;   //
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid2->PID1_RATE_I * 1000;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	_temp = pid2->PID1_RATE_D * 100;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);
	
	NRF24L01_TXDATA[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[_cnt++]=sum;
	
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(NRF24L01_TXDATA,_cnt);
#else
	NRF_TxPacket(NRF24L01_TXDATA,_cnt);
#endif
}

void Data_Send_Check(uint16_t check)
{
	uint8_t sum = 0,i = 0;
	NRF24L01_TXDATA[0]=0xAA;
	NRF24L01_TXDATA[1]=0xAA;
	NRF24L01_TXDATA[2]=0xF0;
	NRF24L01_TXDATA[3]=3;
	NRF24L01_TXDATA[4]=0xBA;
	
	NRF24L01_TXDATA[5]=BYTE1(check);
	NRF24L01_TXDATA[6]=BYTE0(check);
	
	for(i=0;i<7;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[7]=sum;
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(NRF24L01_TXDATA,8);
#else
	NRF_TxPacket(NRF24L01_TXDATA,8);
#endif
}

void Uart1_Put_Buf(uint8_t * tx_buf, uint8_t len)
{
	uint8_t t = 0;
	for(t=0;t<len;t++)
	{
		USART1->DR = *(tx_buf+t) ;
		while((USART1->SR&0X40)==0);//等待发送结束
	}
}

//void Data_Send_DCM(float a,float b ,float c ,float d,float e ,int16_t f)
//{
//	uint8_t _cnt=0;
//	uint8_t sum = 0 ,i = 0;
//	vs16 _temp;
//	NRF24L01_TXDATA[_cnt++]=0xAA;
//	NRF24L01_TXDATA[_cnt++]=0xAA;
//	NRF24L01_TXDATA[_cnt++]=0xF1;
//	NRF24L01_TXDATA[_cnt++]=0;			
//	/**********飞控数据***********/
//	_temp = (vs16)a ;	
//	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
//	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);	
//	
//	_temp = (vs16)b;
//	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
//	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);	
//	
//	_temp = (vs16)c ;	
//	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
//	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);	
//	
//	_temp = (vs16)d ;	
//	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
//	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);	

//	_temp = (vs16)e ;	
//	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
//	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);	
//	
//	_temp = (vs16)f ;	
//	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
//	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);		
//	
//	/***数据总数发往飞控，校验位***/
//	NRF24L01_TXDATA[3] = _cnt-4;
//	
//	for(i=0;i<_cnt;i++)
//		sum += NRF24L01_TXDATA[i];
//	
//	NRF24L01_TXDATA[_cnt++]=sum;
//	/********启动2401开始发数据**********/
//#ifdef DATA_TRANSFER_USE_USART
//	Uart1_Put_Buf(NRF24L01_TXDATA,_cnt);
//#else
//	NRF_TxPacket(NRF24L01_TXDATA,_cnt);
//#endif
//}

void Data_Send_DCM(float a,float b ,float c ,float d,vs32 e ,vs32 f)
{
	uint8_t _cnt=0;
	uint8_t sum = 0 ,i = 0;
	vs16 _temp;
	vs32 _temp2;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0xAA;
	NRF24L01_TXDATA[_cnt++]=0xF1;
	NRF24L01_TXDATA[_cnt++]=0;			
	/**********飞控数据***********/
	_temp = (vs16)a ;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);	
	
	_temp = (vs16)b;
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);	
	
	_temp = (vs16)c ;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);	
	
	_temp = (vs16)d ;	
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp);	

	_temp2 = (vs32)e ;	
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp2);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp2);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp2);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp2);	
	
	_temp2 = (vs32)f ;	
	NRF24L01_TXDATA[_cnt++]=BYTE3(_temp2);
	NRF24L01_TXDATA[_cnt++]=BYTE2(_temp2);
	NRF24L01_TXDATA[_cnt++]=BYTE1(_temp2);
	NRF24L01_TXDATA[_cnt++]=BYTE0(_temp2);		
	
	/***数据总数发往飞控，校验位***/
	NRF24L01_TXDATA[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += NRF24L01_TXDATA[i];
	
	NRF24L01_TXDATA[_cnt++]=sum;
	/********启动2401开始发数据**********/
#ifdef DATA_TRANSFER_USE_USART
	Uart1_Put_Buf(NRF24L01_TXDATA,_cnt);
#else
	NRF_TxPacket(NRF24L01_TXDATA,_cnt);
#endif
}
