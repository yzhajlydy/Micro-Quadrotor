/**********************************************************
HAL.c
主要用于芯片硬件的内部外围和外部外围的初始化，两大INIT函数
在MAIN中调用，使MAIN函数中尽量与硬件库无关
***********************************************************/
#include "stm32f10x.h"
#include "HAL.H"

/*****************************
函数名：void ChipHalInit(void)
说明：片内硬件初始化
入口：无
出口：无
备注：无
*****************************/
void ChipHal_Init(void)
{
	RCC_Configuration();		// 初始化RCC
	NVIC_Configuration();		// 初始化NVIC
	GPIO_Configuration();		// 初始化GPIO
	TIMx_Configuration();		// 初始化TIM
	USART_Configuration();		// 初始化USART
	SPI_Configuration();		// 初始化SPI
	Delay_Init();				// 延迟函数初始化
	IIC_Init();					// 初始化IIC

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);	// 失能JTAG，使能SW-DP
}

/********************************
函数名：void ChipOutHalInit(void)
说明：片外硬件初始化
入口：无
出口：无
备注：无
********************************/
void ChipOutHal_Init(void)
{
	MPU6050_Init();				// MPU6050初始化
	NRF24L01_Init();			// NRF24L01初始化
	while(NRF24L01_Check())		// 检查NRF24L01是否存在,如果不存在，4个蓝色LED同时闪烁
	{
		LED1 = LED2 = LED3 = LED4 = 0;
		Delay_ms(500);
		LED1 = LED2 = LED3 = LED4 = 1;
		Delay_ms(500);
	}
}
