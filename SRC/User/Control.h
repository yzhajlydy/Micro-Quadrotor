#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f10x.h"

//-----油门最大值-----//
#define BASEPWM_MAX		1600	// 限制他的原因是因为如果油门过大，PID调整值超过了PWM最大值，会导致反转力矩不足，侧翻

//-----PID积分最大值-----//
#define PITCH_I_MAX	300
#define ROLL_I_MAX	300

//-----姿态误差结构体声明-----//
struct _Attitude_Error_Tag
{
 float g_Error_Pitch;		// 当前倾仰角误差
 float g_Error_Roll;		// 当前横滚角误差
 float g_Error_Yaw;			// 当前偏航角误差
 float g_ErrorI_Pitch;		// 当前倾仰角误差积分项
 float g_ErrorI_Roll;		// 当前横滚角误差积分项
};

//-----变量声明-----//
extern __IO s16 g_Exp_Pitch;	// 通过遥控给出的微调控制角度
extern __IO s16 g_Exp_Roll;
//-----函数声明-----//
void Quadrotor_Control(const float Exp_Pitch, const float Exp_Roll, const float Exp_Yaw);	// 四旋翼控制函数，用于PWM计算和输出
#endif
