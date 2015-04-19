#include "stm32f10x.h"
#include "PWM.H"

/*************************************
函数名：void TIMx_Configuration(void)
说明：TIMx配置
入口：无
出口：无
备注：无
*************************************/
void TIMx_Configuration(void)
{
	u16 prescalerValue = 0, ccr1_PWMVal = 0;
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;								//TIM定时器初始化结构体	
	TIM_OCInitTypeDef  TIM_OCInitStructure;										//TIM定时器输出比较结构体	
	prescalerValue = (u16) (SystemCoreClock / 2000000) - 1;						//预分频值，时钟基准为72MHz，计算式为时钟基准/(prescalerValue + 1)

	//-----TIM2定时配置-----//
	TIM_TimeBaseStructure.TIM_Period = 2000;									// 2000/2M=1ms，从0开始计数,这个值被写入到Auto-Reload Register中
	TIM_TimeBaseStructure.TIM_Prescaler = 0;									// 暂时不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;								// 时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;					// 向上计数模式
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;							// 专用与TIM1和TIM8，重复比较次数更新事件，我的理解是延长了定时时间
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_PrescalerConfig(TIM2, prescalerValue, TIM_PSCReloadMode_Immediate);		// 预分频,现在计时器频率为2MHz
	
	prescalerValue = (u16) (SystemCoreClock / 20000000) - 1;
	//-----TIM3定时配置-----//
	TIM_TimeBaseStructure.TIM_Period = MAX_PWM;									// 2000/20M=0.1ms-->10KHz，从0开始计数,这个值被写入到Auto-Reload Register中
	TIM_TimeBaseStructure.TIM_Prescaler = 0;									// 暂时不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;								// 时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;					// 向上计数模式
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;							// 专用与TIM1和TIM8，重复比较次数更新事件，我的理解是延长了定时时间
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_PrescalerConfig(TIM3, prescalerValue, TIM_PSCReloadMode_Immediate);		// 预分频,现在计时器频率为20MHz
	
	//-----PWM配置-----//
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 							// 选择定时器模式:TIM脉冲宽度调制模式1-->向上计数为有效电平
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 				// 比较输出使能
	TIM_OCInitStructure.TIM_Pulse = ccr1_PWMVal;								// duty cycle
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 					// 输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);  									// 初始化外设TIM3 OC1-->Motor1
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);  									// 初始化外设TIM3 OC2-->Motor2
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);  									// 初始化外设TIM3 OC3-->Motor3
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);  									// 初始化外设TIM3 OC4-->Motor4

	//-----其他配置-----//
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_ARRPreloadConfig(TIM3, ENABLE);											// 自动重载寄存器使能，下一个更新事件自动更新影子寄存器
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);  							// 使能TIM3在CCR2上的预装载寄存器,在更新事件时，值才被写入到CCR
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);									// 打开更新事件中断
                                                                                   
	//-----使能-----//
	TIM_Cmd(TIM2, ENABLE);														// 使能TIM2
	TIM_Cmd(TIM3, ENABLE);														// 使能TIM3
}
