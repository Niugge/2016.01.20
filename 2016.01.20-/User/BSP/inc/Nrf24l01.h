#ifndef __NRF24L01_H
#define __NRF24L01_H
#include "stm32f10x.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "spi.h"				//nrf24l01��Ҫspi.h�е�uint8_t Spi_RW(uint8_t dat)����
#include "tim_pwm_in.h"
#include "led.h"
//***************************************NRF24L01�Ĵ���ָ��*******************************************************
#define NRF_READ_REG    0x00  	// ���Ĵ���ָ��
#define NRF_WRITE_REG   0x20 	// д�Ĵ���ָ��
#define R_RX_PL_WID   	0x60
#define RD_RX_PLOAD     0x61  	// ��ȡ��������ָ��
#define WR_TX_PLOAD     0xA0  	// д��������ָ��
#define WR_TX_PLOAD2    0xA8  	// д��������ָ��
#define FLUSH_TX        0xE1 	// ��ϴ���� FIFOָ��
#define FLUSH_RX        0xE2  	// ��ϴ���� FIFOָ��
#define REUSE_TX_PL     0xE3  	// �����ظ�װ������ָ��
#define NOP             0xFF  	// ����
//*************************************SPI(nRF24L01)�Ĵ�����ַ****************************************************
#define CONFIG          0x00  // �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
#define EN_AA           0x01  // �Զ�Ӧ��������
#define EN_RXADDR       0x02  // �����ŵ�����
#define SETUP_AW        0x03  // �շ���ַ�������
#define SETUP_RETR      0x04  // �Զ��ط���������
#define RF_CH           0x05  // ����Ƶ������
#define RF_SETUP        0x06  // �������ʡ����Ĺ�������
#define NRFRegSTATUS    0x07  // ״̬�Ĵ���
#define OBSERVE_TX      0x08  // ���ͼ�⹦��
#define CD              0x09  // ��ַ���           
#define RX_ADDR_P0      0x0A  // Ƶ��0�������ݵ�ַ
#define RX_ADDR_P1      0x0B  // Ƶ��1�������ݵ�ַ
#define RX_ADDR_P2      0x0C  // Ƶ��2�������ݵ�ַ
#define RX_ADDR_P3      0x0D  // Ƶ��3�������ݵ�ַ
#define RX_ADDR_P4      0x0E  // Ƶ��4�������ݵ�ַ
#define RX_ADDR_P5      0x0F  // Ƶ��5�������ݵ�ַ
#define TX_ADDR         0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  // ����Ƶ��0�������ݳ���
#define RX_PW_P1        0x12  // ����Ƶ��1�������ݳ���
#define RX_PW_P2        0x13  // ����Ƶ��2�������ݳ���
#define RX_PW_P3        0x14  // ����Ƶ��3�������ݳ���
#define RX_PW_P4        0x15  // ����Ƶ��4�������ݳ���
#define RX_PW_P5        0x16  // ����Ƶ��5�������ݳ���
#define FIFO_STATUS     0x17  // FIFOջ��ջ��״̬�Ĵ�������
//**************************************************************************************
//*********************************************NRF24L01*************************************
#define RX_DR			6	  //�жϱ�־
#define TX_DS			5
#define MAX_RT			4

#define STATUS          0x07  //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�

#define MODEL_RX		1	  //��ͨ����
#define MODEL_TX		2	  //��ͨ����
#define MODEL_RX2		3	  //����ģʽ2,����˫����
#define MODEL_TX2		4	  //����ģʽ2,����˫����

#define RX_PLOAD_WIDTH  32  	
#define TX_PLOAD_WIDTH  32  	
#define TX_ADR_WIDTH    5 	 	
#define RX_ADR_WIDTH    5 

#define MAX_TX  		0x10  //�ﵽ����ʹ����ж�
#define TX_OK   		0x20  //TX��������ж�
#define RX_OK   		0x40  //���յ������ж�
extern 	uint8_t 	NRF24L01_RXDATA[RX_PLOAD_WIDTH];		//nrf24l01���յ�������
extern 	uint8_t 	NRF24L01_TXDATA[RX_PLOAD_WIDTH];		//nrf24l01��Ҫ���͵�����

#define NRF_IRQ (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_5))

void Nrf24l01_Init(uint8_t model, uint8_t ch);				//��ʼ��,model=1/2/3/4,chΪʵ�õ�ͨ����
void NRF_TxPacket(uint8_t * tx_buf, uint8_t len);	//�������ݰ�,����model 2/4
void NRF_TxPacket_AP(uint8_t * tx_buf, uint8_t len);//�������ݰ�,����model 3
uint8_t NRF_Read_Reg(uint8_t reg);
uint8_t NRF_Write_Reg(uint8_t reg, uint8_t value);
uint8_t NRF_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t uchars);
uint8_t Nrf24l01_Check(void);
uint8_t Nrf_Get_FIFOSta(void);
#endif


