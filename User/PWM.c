#include "stm32f10x.h"
#include "HAL.H"

/***********************************************************************************
函数名：void PWM_Set(const u16 pwm1, const u16 pwm2, const u16 pwm3, const u16 pwm4)
说明：PWM设置
入口：无
出口：无
备注：根据PWM的数值给与相应PWM信号灯的状态:
当PWM为0时，灯灭
不为0时，灯亮
************************************************************************************/
void PWM_Set(const u16 pwm1, const u16 pwm2, const u16 pwm3, const u16 pwm4)
{
	if (pwm1 == 0)	//没有PWM，此时灯灭
		LED1 = 1;
	else
		LED1 = 0;
	if (pwm2 == 0)	//没有PWM，此时灯灭
		LED2 = 1;
	else
		LED2 = 0;
	if (pwm3 == 0)	//没有PWM，此时灯灭
		LED3 = 1;
	else
		LED3 = 0;
	if (pwm4 == 0)	//没有PWM，此时灯灭
		LED4 = 1;
	else
		LED4 = 0;
	TIM_SetCompare1(TIM3, pwm1);
	TIM_SetCompare2(TIM3, pwm2);
	TIM_SetCompare3(TIM3, pwm3);
	TIM_SetCompare4(TIM3, pwm4);
}
