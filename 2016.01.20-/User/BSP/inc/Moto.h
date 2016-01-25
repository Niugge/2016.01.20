#ifndef _BSP_MOTO_H
#define _BSP_MOTO_H
#include "stm32f10x.h"

#define Moto_PwmMax 1999
#define Moto_PwmMin 999

void Moto_PwmRflash(MOTO_PWM *moto_pwm);
void Moto_Init(void);
void Tim5_init(void);

#endif
