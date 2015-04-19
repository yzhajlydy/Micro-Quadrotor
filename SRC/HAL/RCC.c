#include "stm32f10x.h"

//-----定义时钟结构体-----//
RCC_ClocksTypeDef RCC_ClockFreq;

/*************************************
函数名：void RCC_Configuration(void)
说明：RCC时钟配置
入口：无
出口：无
备注：无
*************************************/
void RCC_Configuration(void)
{
//	SystemInit();//源自system_stm32f10x.c文件,如果需要更改PLL等设置，这里可以自定义时钟初始化函数再初始化一遍
	//因为在V3.5.0官方库中的启动代码已经将SystemInit()嵌入进去。
//	RCC_PCLK1Config(RCC_HCLK_Div4);		// PCLK1，APB1对AHB总线4分频
	/**************************************************
	获取RCC的信息,调试用
	请参考RCC_ClocksTypeDef结构体的内容,当时钟配置完成后,
	里面变量的值就直接反映了器件各个部分的运行频率
	***************************************************/
	RCC_GetClocksFreq(&RCC_ClockFreq);
	
	/* 这个配置可使外部晶振停振的时候,产生一个NMI中断,不需要用的可屏蔽掉*/
	//RCC_ClockSecuritySystemCmd(ENABLE);
	
	//-----GPIO时钟-----//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | 
													RCC_APB2Periph_GPIOB 
													//RCC_APB2Periph_GPIOC |
													//RCC_APB2Periph_GPIOD | 
													//RCC_APB2Periph_GPIOE |
													//RCC_APB2Periph_GPIOF | 
													/*RCC_APB2Periph_GPIOG*/, ENABLE);		// 使能GPIOA,B,C,D,E,F,G时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);		// 使能复用功能时钟,启用外部中断时，必须开启复用时钟
	//-----SPI2时钟使能-----//
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	//-----TIM3时钟-----//
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	//-----TIM2时钟-----//
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//-----USART1时钟-----//
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	//-----DMA1时钟-----//
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);			// 注意挂接在AHB总线上
}
