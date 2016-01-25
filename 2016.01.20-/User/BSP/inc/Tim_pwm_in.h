#ifndef __TIM_PWM_IN_H
#define __TIM_PWM_IN_H
#include "stm32f10x.h"

void RC_Read(T_RC_Data *temp);
void Tim_Pwm_In_Init(void);
void Tim4_Pwm_In_Irq(void);
void Tim2_Pwm_In_Irq(void);

#endif
