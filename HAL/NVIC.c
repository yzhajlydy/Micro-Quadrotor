#include "stm32f10x.h"

/***********************************
函数名：void NVIC_Configuration(void)
说明：NVIC初始化
入口：无
出口：无
备注：无
************************************/
void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;												//定义NVIC初始化结构体
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);							//优先级组别1，具体参见misc.h line80
	
	//-----NRF24L01数据中断-----//
  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;				//IRQ中断通道-->NRF24L01,PB12
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		//抢先式优先级别
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					//副优先级别
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//使能通道
  NVIC_Init(&NVIC_InitStructure);															//初始化NVIC
	
	//-----TIM2定时中断-----//
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;							//选中TIM2中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		//抢先式优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;					//副优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
