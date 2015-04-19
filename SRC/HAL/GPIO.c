#include "stm32f10x.h"

/*************************************
函数名：void GPIO_Configuration(void)
说明：GPIO配置
入口：无
出口：无
备注：无
*************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;		// 定义GPIO结构体
	EXTI_InitTypeDef EXTI_InitStructure;		// 定义EXTI结构体

	//------PWM_LED------//
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;	// PB6,7,8,9-->PWM_LED
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;									// 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;									// 50MHz翻转频率
	GPIO_Init(GPIOB, &GPIO_InitStructure);												// 初始化IO
	GPIO_SetBits(GPIOB, GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
	
	//------LED------//
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;	// PA0,1,2,3,4-->LED
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;									// 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;									// 50MHz翻转频率
	GPIO_Init(GPIOA, &GPIO_InitStructure);												// 初始化IO
	GPIO_SetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
	
	//------IIC------//
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;							// PA11,12-->IIC
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;									// 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;									// 50MHz翻转频率
	GPIO_Init(GPIOA, &GPIO_InitStructure);												// 初始化IO
	
	//------PWM------//
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;										// 复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;									// 50MHz翻转频率
	GPIO_Init(GPIOA, &GPIO_InitStructure);												// 初始化IO
	GPIO_ResetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;										// 复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;									// 50MHz翻转频率
	GPIO_Init(GPIOB, &GPIO_InitStructure);												// 初始化IO
	GPIO_ResetBits(GPIOB, GPIO_Pin_0 | GPIO_Pin_1);
	
	//-----NRF24L01-----//
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;							// PB10,11-->NRF_CE,NRF_CSN
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;									// 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;									// 50MHz翻转频率
	GPIO_Init(GPIOB, &GPIO_InitStructure);												// 初始化IO
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;   										// PB12-->NRF_IRQ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;									// 50MHz翻转频率
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 										// 输入下拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//-----SPI2-----//
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;				// PB13-->SCK,PB14-->MISO,PB15-->MOSI
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  									// PB13/14/15复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//-----USART1-----//
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;											// PA9复用为发送
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;					
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;										// 复用为推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;											// PA10复用为接收
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;								// 浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//-----中断事件线连接-----//
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);						// 将事件线和Pin连接起来	
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;											// EXTI线中断开通
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;									// 定义为中断还是事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;								// 下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;											// 使能EXTI线中断
	EXTI_Init(&EXTI_InitStructure);
}
