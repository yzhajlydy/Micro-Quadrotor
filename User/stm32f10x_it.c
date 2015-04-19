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
#include "HAL.H"

//-----外部变量声明-----//
extern __IO u8 g_ConCycT_flag;
extern __IO u8 g_Fly_Enable;
extern __IO s16 g_BasePWM;

//-----定时器2计数器-----//
u16 g_tim2counter = 0;

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

/**
  * @}
  */ 

/***********************************
函数名：void TIM2_IRQHandler(void)
说明：TIM2中断处理函数
入口：无
出口：无
备注：用于系统定时，给出控制周期
***********************************/
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)	// 更新中断,1ms进来一次
	{
		g_tim2counter++;
		if (g_tim2counter % 500 == 0)	// 500ms,翠绿灯闪烁，表明系统正常运行
			PWM3_LED = !PBin(8);
		if (g_tim2counter % 3 == 0)		// 3ms中断控制周期
			g_ConCycT_flag = 1;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);		// 清除中断标志位
	}
}

/***************************************
函数名：void EXTI15_10_IRQHandler(void)
说明：EXTI15_10_IRQ组别中断处理函数
入口：无
出口：无
备注：用于接收NRF24L01数据
***************************************/
void EXTI15_10_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line12) != RESET)
  {
		if (NRF24L01_RxPacket(g_NRFRevData) == 0)	// 接收成功
		{
			g_NRFRevCnt++;	// 计数器++,计数器在计数,就说明遥控正在发送数据进来
			if (g_NRFRevData[0] == 239)	// 起飞允许
				g_Fly_Enable = 1;
			if (g_NRFRevData[0] == 223)	// 起飞失能
				g_Fly_Enable = 0;
			if (g_NRFRevData[0] == 127)	// 加油门
			{
				g_BasePWM += 20;
				if (g_BasePWM >= BASEPWM_MAX)		// 大于最大油门限制
					g_BasePWM = BASEPWM_MAX;
			}
			if (g_NRFRevData[0] == 191)	// 减油门
			{
				g_BasePWM -= 20;
				if (g_BasePWM <= 0)
					g_BasePWM = 0;
			}
			if (g_NRFRevData[0] == 253)	// 左滚
			{
				g_Exp_Roll = -8;					// -8°
				g_Exp_Pitch = 0;
			}
			if (g_NRFRevData[0] == 254)	// 右滚
			{
				g_Exp_Roll = 8;						// +8°
				g_Exp_Pitch = 0;
			}
			if (g_NRFRevData[0] == 247)	// 前倾
			{
				g_Exp_Pitch = -8;					// -8°
				g_Exp_Roll = 0;
			}
			if (g_NRFRevData[0] == 251)	// 后仰
			{
				g_Exp_Pitch = 8;					// +8°
				g_Exp_Roll = 0;
			}
				
		}
    EXTI_ClearITPendingBit(EXTI_Line12);			// 清除标志位
  }	
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
