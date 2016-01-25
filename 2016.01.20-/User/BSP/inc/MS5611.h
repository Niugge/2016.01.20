#ifndef __MS5611_H
#define __MS5611_H

#include "MPU6050.h"
#include "math.h"
#include "usart.h"
#include "delay.h"
#include "IMU.h"
#include "sys.h"
/*====================================================================================================*/
/*====================================================================================================*/
#define BARO_CAL_CNT 100
#define MS561101BA_SlaveAddress   0xEE    //定义器件在IIC总线中的从地址0XEE
#define MS561101BA_RST 				    0x1E 
#define MS5611_ADC           ((uint8_t)0x00)   // ADC Read
#define MS561101BA_D1_OSR_4096    0x48 		//气压
#define MS561101BA_D2_OSR_4096    0x58 		//温度

#define MS561101BA_PROM_CRC 			0xAE
//气压
//#define MS5611_D1_OSR_256     ((uint8_t)0x40)  // 3 bytes
//#define MS5611_D1_OSR_512     ((uint8_t)0x42)  // 3 bytes
//#define MS5611_D1_OSR_1024    ((uint8_t)0x44)  // 3 bytes
//#define MS5611_D1_OSR_2048    ((uint8_t)0x46)  // 3 bytes
#define MS5611_D1_OSR_4096    ((uint8_t)0x48)  // 3 bytes
//温度
//#define MS5611_D2_OSR_256     ((uint8_t)0x50)  // 3 bytes
//#define MS5611_D2_OSR_512     ((uint8_t)0x52)  // 3 bytes
//#define MS5611_D2_OSR_1024    ((uint8_t)0x54)  // 3 bytes
//#define MS5611_D2_OSR_2048    ((uint8_t)0x56)  // 3 bytes
#define MS5611_D2_OSR_4096    ((uint8_t)0x58)  // 3 bytes
//出厂参数
#define MS5611_PROM_COEFF_1   ((uint8_t)0xA2)  // 2 bytes
#define MS5611_PROM_COEFF_2   ((uint8_t)0xA4)  // 2 bytes
#define MS5611_PROM_COEFF_3   ((uint8_t)0xA6)  // 2 bytes
#define MS5611_PROM_COEFF_4   ((uint8_t)0xA8)  // 2 bytes
#define MS5611_PROM_COEFF_5   ((uint8_t)0xAA)  // 2 bytes
#define MS5611_PROM_COEFF_6   ((uint8_t)0xAC)  // 2 bytes

#define MS5611_ADC_D1         ((uint8_t)0x01)
#define MS5611_ADC_D2         ((uint8_t)0x02)

#define MS5611_RespFreq_256   ((uint16_t)1650) // 0.48 - 0.54 - 0.60 - 1650Hz
#define MS5611_RespFreq_512   ((uint16_t)850)  // 0.95 - 1.06 - 1.17 - 850Hz
#define MS5611_RespFreq_1024  ((uint16_t)430)  // 1.88 - 2.08 - 2.28 - 430Hz
#define MS5611_RespFreq_2048  ((uint16_t)220)  // 3.72 - 4.13 - 4.54 - 220Hz
#define MS5611_RespFreq_4096  ((uint16_t)110)  // 7.40 - 8.22 - 9.04 - 110Hz
 

uint16_t MS5611_ReadByte_16b(uint8_t SlaveAddress,uint8_t REG_Address);
uint8_t  MS5611_ReadByte_8b(uint8_t SlaveAddress,uint8_t REG_Address);
uint8_t  MS561101BA_RESET(void); 
u32 MS561101BA_ADC_CONVERSION(uint8_t command); 
void MS561101BA_PROM_READ(void); 
void MS561101BA_Init(void); 
void MS5611_Read(vs32 *alt); 


#endif




