#include "nrf24l01.h"

	
uint8_t NRF24L01_RXDATA[RX_PLOAD_WIDTH];//nrf24l01���յ�������
uint8_t NRF24L01_TXDATA[RX_PLOAD_WIDTH];//nrf24l01��Ҫ���͵�����

uint8_t  TX_ADDRESS[TX_ADR_WIDTH]= {0xAA,0xBB,0xCC,0x00,0x09};	//���ص�ַ
uint8_t  RX_ADDRESS[RX_ADR_WIDTH]= {0xAA,0xBB,0xCC,0x00,0x09};	//���յ�ַ

/*
*****************************************************************
* д�Ĵ���
*****************************************************************
*/
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;
	SPI_CSN_L();					 /* ѡͨ���� */
	status = Spi_RW(reg);  /* д�Ĵ�����ַ */
	Spi_RW(value);		     /* д���� */
	SPI_CSN_H();					 /* ��ֹ������ */
  return 	status;
}
/*
*****************************************************************
* ���Ĵ���
*****************************************************************
*/
uint8_t NRF_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	SPI_CSN_L();			  /* ѡͨ���� */
	Spi_RW(reg);			  /* д�Ĵ�����ַ */
	reg_val = Spi_RW(0);	  /* ��ȡ�üĴ����������� */
	SPI_CSN_H();			  /* ��ֹ������ */
    return 	reg_val;
}
/*
*****************************************************************
*
* д������
*
*****************************************************************
*/
uint8_t NRF_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	SPI_CSN_L();				        /* ѡͨ���� */
	status = Spi_RW(reg);	/* д�Ĵ�����ַ */
	for(i=0; i<uchars; i++)
	{
		Spi_RW(pBuf[i]);		/* д���� */
	}
	SPI_CSN_H();						/* ��ֹ������ */
    return 	status;	
}
/*
*****************************************************************
* ��������
*****************************************************************
*/
uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars)
{
	uint8_t i;
	uint8_t status;
	SPI_CSN_L();						/* ѡͨ���� */
	status = Spi_RW(reg);	/* д�Ĵ�����ַ */
	for(i=0; i<uchars; i++)
	{
		pBuf[i] = Spi_RW(0); /* ��ȡ�������� */ 	
	}
	SPI_CSN_H();						/* ��ֹ������ */
    return 	status;
}
/*
*****************************************************************
* д���ݰ�
*****************************************************************
*/
void NRF_TxPacket(uint8_t * tx_buf, uint8_t len)
{	
	uint8_t sta = 0;
	static uint8_t i = 0;
	volatile uint16_t cnt = 0;
	SPI_CE_L();		 //StandBy Iģʽ		
	NRF_Write_Buf(WR_TX_PLOAD2, tx_buf, len); // װ������	
	SPI_CE_H();		 //�ø�CE���������ݷ���
	sta=NRF_Read_Reg(STATUS);//read state register
//	while(NRF_IRQ != 0)//�ȴ��������	
//	{
//		cnt++;
//		if(cnt > 5000)
//		{
//			Nrf24l01_Init(MODEL_TX2,40);
//			return;	
//		}			
//	}	
	NRF_Write_Reg(NRF_WRITE_REG+STATUS,sta); //���TX_DS��MAX_RT�жϱ�־
	if(sta&MAX_TX)//�ﵽ����ط�����
	{
		NRF_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ��� 
	}
	if(sta&TX_OK)//�������
	{			
		i++;
		if(i == 40)
		{
			yellow_toggle();
			i = 0;				
		}			
	}	
}


void NRF_TxPacket_AP(uint8_t * tx_buf, uint8_t len)
{		
	SPI_CE_L();		 //StandBy Iģʽ	
	NRF_Write_Buf(WR_TX_PLOAD2, tx_buf, len); 			 // װ������	
	SPI_CE_H();		 //�ø�CE���������ݷ���
}

uint8_t Nrf24l01_Check(void)
{ 
	uint8_t buf1[5]; 
	uint8_t i; 
	/*д��5���ֽڵĵ�ַ. */ 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,5); 
	/*����д��ĵ�ַ */ 
	NRF_Read_Buf(TX_ADDR,buf1,5); 
	/*�Ƚ�*/ 
	for(i=0;i<5;i++) 
	{ 
		if(buf1[i]!=TX_ADDRESS[i]) 
			break; 
	} 
	if(i==5)
		return SUCCESS ; //MCU��NRF�ɹ����� 
	else
		return ERROR ; //MCU��NRF���������� 
}
void Nrf24l01_Init(uint8_t model, uint8_t ch)
{
	//Spi_Change_To_nRF24L01_();
	SPI_CE_L();
	NRF_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,RX_ADDRESS,RX_ADR_WIDTH);	//дRX�ڵ��ַ 
	NRF_Write_Buf(NRF_WRITE_REG+TX_ADDR,TX_ADDRESS,TX_ADR_WIDTH); 		//дTX�ڵ��ַ  
	NRF_Write_Reg(NRF_WRITE_REG+EN_AA,0x01); 													//ʹ��ͨ��0���Զ�Ӧ�� 
	NRF_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01);											//ʹ��ͨ��0�Ľ��յ�ַ 
	NRF_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);											//�����Զ��ط����ʱ��:500us;����Զ��ط�����:10�� 
	NRF_Write_Reg(NRF_WRITE_REG+RF_CH,ch);														//����RFͨ��ΪCHANAL
	NRF_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f); 												//����TX�������,0db����,2Mbps,���������濪��
	if(model==1)				//RX
	{
		NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//ѡ��ͨ��0����Ч���ݿ�� 
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ�շ�����жϿ���,16λCRC,������
	}
	else if(model==2)		//TX
	{
		NRF_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//ѡ��ͨ��0����Ч���ݿ�� 
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ�շ�����жϿ���,16λCRC,������
	}
	else if(model==3)		//RX2
	{
		NRF_Write_Reg(FLUSH_TX,0xff);
		NRF_Write_Reg(FLUSH_RX,0xff);
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);   		 // IRQ�շ�����жϿ���,16λCRC,������
		
		Spi_RW(0x50);
		Spi_RW(0x73);
		NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
	}
	else								//TX2
	{
		NRF_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ�շ�����жϿ���,16λCRC,������
		NRF_Write_Reg(FLUSH_TX,0xff);
		NRF_Write_Reg(FLUSH_RX,0xff);
		
		Spi_RW(0x50);
		Spi_RW(0x73);
		NRF_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
	}
	SPI_CE_H();
}

uint8_t Nrf_Get_FIFOSta(void)
{
	return NRF_Read_Reg(NRF_READ_REG + FIFO_STATUS);
}

