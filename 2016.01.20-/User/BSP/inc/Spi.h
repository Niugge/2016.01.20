#ifndef _SPI_H
#define _SPI_H
#include "stm32f10x.h"
//PA4 2401_CE
#define SPI_CE_H()   GPIO_SetBits(GPIOA, GPIO_Pin_4) 
#define SPI_CE_L()   GPIO_ResetBits(GPIOA, GPIO_Pin_4)
//PC4 2401_CS
#define SPI_CSN_H()  GPIO_SetBits(GPIOC, GPIO_Pin_4)
#define SPI_CSN_L()  GPIO_ResetBits(GPIOC, GPIO_Pin_4)

//ADNS3080Ƭѡ,PC3
#define  ON_CS()    GPIO_ResetBits(GPIOC,GPIO_Pin_6)
#define  OFF_CS()   GPIO_SetBits(GPIOC,GPIO_Pin_6)

#define MISO   GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)//

#define SCK_H()    GPIO_SetBits(GPIOC, GPIO_Pin_9)
#define SCK_L()    GPIO_ResetBits(GPIOC, GPIO_Pin_9)
#define MOSI_H()   GPIO_SetBits(GPIOC, GPIO_Pin_8)
#define MOSI_L()   GPIO_ResetBits(GPIOC, GPIO_Pin_8)

void Spi1_Init(void);
void SPI_Simu_Init(void);
u8 SPI_Simu_RW(uint8_t dat);
uint8_t Spi_RW(uint8_t dat);
void Spi_Change_To_ADNS3080_(void);
void Spi_Change_To_nRF24L01_(void);

#endif
