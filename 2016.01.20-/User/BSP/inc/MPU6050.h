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
// 定义MPU6050内部地址
/****************************************/
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	MPU_CONFIG		0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
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

#define INT_PIN_CFG     0x37    //设置旁路有效 打开值：0x42 AUX_DA的辅助I2C
#define USER_CTRL       0x6A    //用户配置寄存器 打开值：0x40  AUX_DA的辅助I2C


#define	PWR_MGMT_1	 0x6B	  //电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I	 0x75	  //IIC地址寄存器(默认数值0x68，只读)
#define	MPU6050_Addr 0xD0	  //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改  特征值 0XD0 0XD2

/*****6050接口******/
void MPU6050_Read(MPU6050_DATA *sensor_data);
void MPU6050_Init(void);
uint8_t MPU6050_WriteByte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
uint8_t MPU6050_ReadByte(uint8_t SlaveAddress,uint8_t REG_Address);
void MPU6050_AcknowledgePolling(uint8_t Addr);
void MPU6050_setI2CMasterModeEnabled(void);
void MPU6050_setI2CBypassEnabled(void);
/*****IIC接口******/
uint8_t I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
uint8_t I2C_WaitAck(void);
void I2C_NoAck(void);
void I2C_SendByte(uint8_t SendByte);
uint8_t I2C_ReadByte(void);
void I2C_delay(void);



#endif

