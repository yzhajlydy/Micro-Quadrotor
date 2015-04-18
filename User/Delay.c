#include "stm32f10x.h"
#include "Delay.H"

//-----私有全局变量-----//
static u8  g_fac_us = 0;			//us定时因子
static u16 g_fac_ms = 0;			//ms定时因子

/****************************************************
函数名：void Delay_Init(void)
说明：延迟初始化
入口：无
出口：无
备注：使用SysTick时钟进行延迟，请务必使用8MHz外部晶振
*****************************************************/
void Delay_Init(void)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟HCLK/8
	g_fac_us = SystemCoreClock / 8000000;									//时钟的1/8  
	g_fac_ms = (u16)g_fac_us * 1000;											//延迟因子
}	

/****************************************************
函数名：void Delay_us(u32 nus)
说明：us延迟
入口：u32 nus  输入延迟的us，最大范围为(2^24/fac_us)
出口：无
备注：使用SysTick时钟进行延迟，请务必使用8MHz外部晶振
*****************************************************/ 								   
void Delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD = nus * g_fac_us;	 
	SysTick->VAL = 0x00;        									//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk ;    //开始倒数
	do
	{
		temp = SysTick->CTRL;
	}
	while (temp & 0x01 && !(temp & (1 << 16)));				//等待时间到达
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;        //关闭计数器
	SysTick->VAL = 0x00;     													//清空计数器
}
/**************************************************************
函数名：void Delay_ms(u16 nms)
说明：ms延迟
入口16 nms  输入延迟的ms，最大范围为(0xffffff*8*1000/SYSCLK)
出口：无
备注：8MHz时的最大延迟为1864ms
***************************************************************/ 		
void Delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD = (u32)nms*g_fac_ms;								//时间加载，SysTick->LOAD为24bit
	SysTick->VAL = 0x00;          										//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}
	while (temp & 0x01 && !(temp & (1 << 16)));				//等待时间到达
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;				//关闭计数器
	SysTick->VAL = 0x00;     													//清空计数器
} 
