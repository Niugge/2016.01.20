#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x_gpio.h"
#include "sys.h"
#include "flash.h"
#include "eeprom.h"
/********************************************/
#ifdef USE_BIG_SIZE_BOARD
/***scl PC12***/
#define SCL_L (GPIOC->BRR  = GPIO_Pin_12)
#define SCL_H (GPIOC->BSRR = GPIO_Pin_12)
/***sda PC11***/
#define SDA_L (GPIOC->BRR  = GPIO_Pin_11)
#define SDA_H (GPIOC->BSRR = GPIO_Pin_11)

#define SCL_read      (GPIOC->IDR  & GPIO_Pin_12)
#define SDA_read      (GPIOC->IDR  & GPIO_Pin_11)

#endif
/***********************************************/
#ifdef USE_LITTLE_SIZE_BOARD
/***scl PB15***/
#define SCL_L (GPIOB->BRR  = GPIO_Pin_15)
#define SCL_H (GPIOB->BSRR = GPIO_Pin_15)
/***sda PB13***/
#define SDA_L (GPIOB->BRR  = GPIO_Pin_13)
#define SDA_H (GPIOB->BSRR = GPIO_Pin_13)

#define SCL_read  (GPIOB->IDR  & GPIO_Pin_15)
#define SDA_read  (GPIOB->IDR  & GPIO_Pin_13)

#endif
/**********************************************/
// ����MPU6050�ڲ���ַ
/****************************************/
#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	MPU_CONFIG		0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_INT_PIN_CFG      0x37


#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_USERCTRL_I2C_MST_EN_BIT     5

#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1

#define INT_PIN_CFG     0x37    //������·��Ч ��ֵ��0x42 AUX_DA�ĸ���I2C
#define USER_CTRL       0x6A    //�û����üĴ��� ��ֵ��0x40  AUX_DA�ĸ���I2C


#define	PWR_MGMT_1	 0x6B	  //��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I	 0x75	  //IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define	MPU6050_Addr 0xD0	  //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�  ����ֵ 0XD0 0XD2

/*****6050�ӿ�******/
void MPU6050_Read(MPU6050_DATA *sensor_data);
void MPU6050_Init(void);
uint8_t MPU6050_WriteByte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
uint8_t MPU6050_ReadByte(uint8_t SlaveAddress,uint8_t REG_Address);
void MPU6050_AcknowledgePolling(uint8_t Addr);
void MPU6050_setI2CMasterModeEnabled(void);
void MPU6050_setI2CBypassEnabled(void);
/*****IIC�ӿ�******/
uint8_t I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
uint8_t I2C_WaitAck(void);
void I2C_NoAck(void);
void I2C_SendByte(uint8_t SendByte);
uint8_t I2C_ReadByte(void);
void I2C_delay(void);



#endif

