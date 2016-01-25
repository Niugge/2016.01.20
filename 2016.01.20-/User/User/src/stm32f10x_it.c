/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "DATA_Transfer.h"
#include "tim_pwm_in.h"
#include "nrf24l01.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "timer.h"
#include "usart.h"
#include "adc.h"
#include "led.h"
#include "IMU.h"
#include "PID.h"
#include "sys.h"
#include "Altitude.h"
#include "filter.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/****遥控输入捕获中断*****/
void TIM4_IRQHandler(void)		
{	
	Tim4_Pwm_In_Irq();
}
void TIM2_IRQHandler(void)		
{	
	Tim2_Pwm_In_Irq();
}
/*********************************
串口接受中断，读取GPS数据
串口1中断服务程序注意,
读取USARTx->SR能避免莫名其妙的错误
**********************************/   	
uint8_t  GPS_RX_BUF[USART_REC_LEN] = {0}; //接收缓冲,最大USART_REC_LEN个字节.
uint16_t USART_RX_STA=0;//接收状态标记	  

#if EN_USART1_RX   //如果使能了接收
void USART1_IRQHandler(void)          
{
	static uint8_t i = 0;
	//发送中断
	if((USART1->SR & (1<<7))&&(USART1->CR1 & USART_CR1_TXEIE))
	{

	}
	//接收中断
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  
	{
		GPS_RX_BUF[i] =USART_ReceiveData(USART1);	//读取接收到的数据		
		i++;
		//过滤GPS数据
		if(i==200) i=0;							
	} 
} 
#endif
/**************************************************/
/***********4ms TIM3中断服务子函数*****************/
void TIM3_IRQHandler(void)  
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
		{			
			static uint8_t IMU_flag = 0,Status_flag = 0,RC_flag = 0,MOto_flag = 0,GPS_flag = 0;
			IMU_flag++;
			Status_flag++;
			RC_flag++;
			MOto_flag++;
			GPS_flag++;
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);//清除TIMx的中断待处理位:TIM 中断源 
			/**************/
			MPU6050_Read(&mpu6050);
			/*****加速度计滤波**********/
			//ACC_Smooth_Filter(&mpu6050,&mpu6050_smooth);
			Acc_Butterworth_Filter(&mpu6050,&mpu6050_Butterworse);
			/**************/
			HMC5883L_Read(&hmc5883l,&hmc5883l_cal);	
			/********************/
			Unit_Unify(&mpu6050_Butterworse,&hmc5883l_cal,&mpu6050_unit,&mpu6050_unit_deg,&hmc5883l_unit);					
			/***read RC_DATA & PID1_data & calibration sensor***/
			Nrf_Check_Event();
			Rc_Fun(&Rc_D,&ARMed);
//////////////////////////////////////////////////////
			/******************IMU get angel********************/
#ifdef USE_IMU			
			IMU_Update(mpu6050_unit,&angel_quad);
			Get_CompassAngle(&hmc5883l_unit,&angel_quad.yaw);
#endif
			/****************************************************/
#ifdef USE_AHRS
			AHRSupdate(mpu6050_unit,hmc5883l_unit,&angel_quad); 
#endif
      /****************************************************/
#ifdef USE_GraDescent
			GraDescent_Updata(mpu6050_unit,&angel_quad);
			Get_CompassAngle(&hmc5883l_unit,&angel_quad.yaw);
#endif
			/****************************************************/
#ifdef USE_GraDescentAll
			GraDescentAll_Updata(mpu6050_unit,hmc5883l_unit,&angel_quad); 
#endif
			/****************************************************/
#ifdef USE_Kalman_IMU			
			//IMU_Update(mpu6050_unit,&angel_quad);
			Kalman_IMU_Update(mpu6050_unit,&angel_quad);
			Get_CompassAngle(&hmc5883l_unit,&angel_quad.yaw);
#endif  
			/****************************************************/
#ifdef USE_Kalman_Grad_IMU			
			//IMU_Update(mpu6050_unit,&angel_quad);
			Kalman_Grad_Update(mpu6050_unit,&angel_quad);
			Get_CompassAngle(&hmc5883l_unit,&angel_quad.yaw);
#endif  
      /****************************************************/
#ifdef USE_Kalman_Grad_ALL_IMU			
			//IMU_Update(mpu6050_unit,&angel_quad1);
			Kalman_Grad_ALL_Update(mpu6050_unit,hmc5883l_unit,&angel_quad);
			//Get_CompassAngle(&hmc5883l_unit,&angel_quad1.yaw);
#endif  
      /****************************************************/
/////////////////////////////////////////////////////////
			/*****************高度**************************/
#ifdef USE_Complementary
		  Altitude_Calculation(BaroAlt,mpu6050_unit, angel_quad,&Pos_rate,&Alt_Estimated);	
#endif
#ifdef USE_Kalman_Complementary
		  Kalman_Comple_Filter_Altitude(BaroAlt,mpu6050_unit, angel_quad,&Pos_rate,&Alt_Estimated);	
#endif
#ifdef USE_Kalman_
		  Kalman_Filter_Altitude(BaroAlt,mpu6050_unit, angel_quad,&Pos_rate,&Alt_Estimated);	
#endif

			Altitude_Control(&Calculate_THR);//高度控制
			/**************/
			Angel_Control(&angel_quad,&mpu6050_unit_deg,&Rc_D,ARMed);	
			/********发送数据********/
			if(Send_PID1 == 1)
			{
				static uint8_t pid_flag;
				pid_flag++;
				switch(pid_flag)
				{
					case 1:
						Data_Send_PID1(&PID1_ipt);
						break;
					case 2:
						Data_Send_PID2(&PID1_ipt);
						break;
					case 3:
						Data_Send_PID3(&PID2_ipt);
						break;
					case 4:
						Data_Send_PID4(&PID2_ipt);	
						pid_flag = 0;					
						Send_PID1 = 0;
						break;
					default:
						break;				
				}
			}
			if(IMU_flag == 1)
				Data_Send_IMU_Data(&mpu6050,&hmc5883l);
			if(Status_flag == 2)
				Data_Send_Status(&angel_quad,&Alt_Estimated,ARMed);
			if(RC_flag == 3)
				Data_Send_RCData(&Rc_D);
			if(MOto_flag == 4)
			{
				Data_Send_MotoPWM(&moto_rate);
				IMU_flag = 0;
				Status_flag = 0;
				RC_flag = 0;
				MOto_flag = 0;
			}
			if(GPS_flag == 5)
			{
				Data_Send_GPSData(&Gps_Data);
				GPS_flag=0;
			}
			/**test data**/
		}	
}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
