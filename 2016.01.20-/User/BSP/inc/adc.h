#ifndef __ADC_H
#define __ADC_H
#include "stm32f10x_adc.h"
#include "stm32f10x_conf.h"
#include "sys.h"
#include "delay.h"
#define bettary_low	11.1f
void Adc_cfig(void);
uint16_t read_adc(uint8_t channel,uint8_t rank);
uint16_t Get_Adc(uint8_t channels,uint8_t ranks); 
#endif

