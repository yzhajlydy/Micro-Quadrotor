#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

//-----最大PWM值-----//
#define MAX_PWM 2000

//-----函数声明-----//
extern void PWM_Set(const u16 pwm1,const u16 pwm2,const u16 pwm3,const u16 pwm4);	// PWM设置

#endif
