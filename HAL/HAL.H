#ifndef __HAL_H__
#define __HAL_H__

#include "Delay.H"
#include "IIC.H"
#include "SPI.H"
#include "PWM.H"
#include "MPU6050.H"
#include "NRF24L01.H"
#include "IO_Bits.H"
#include "Fusion.H"
#include "DMA.H"
#include "USART.H"
#include "Control.H"

//-----接口定义-----//
//-----LED-----//
#define LED1 PAout(0)
#define LED2 PAout(1)
#define LED3 PAout(2)
#define LED4 PAout(3)
//-----PWM_LED-----//
#define PWM1_LED PBout(6)
#define PWM2_LED PBout(7)
#define PWM3_LED PBout(8)
#define PWM4_LED PBout(9)

//-----各个内部硬件模块的配置函数-----//
extern void RCC_Configuration(void);		// RCC配置
extern void GPIO_Configuration(void);		// GPIO配置
extern void NVIC_Configuration(void);		// NVIC配置
extern void ADC_Configuration(void);		// ADC配置
extern void USART_Configuration(void);		// USART配置
extern void TIMx_Configuration(void);		// TIMx配置
extern void Delay_Init(void);				// 延迟初始化

//-----硬件初始化-----//
extern void ChipHal_Init(void);				// 片内硬件初始化
extern void ChipOutHal_Init(void);			// 片外硬件初始化

#endif
