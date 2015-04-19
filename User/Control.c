#include "stm32f10x.h"
#include "math.h"
#include "PWM.H"
#include "Fusion.H"
#include "Control.H"
#include "HAL.H"

//-----起飞确认-----//
__IO u8 g_Fly_Enable = 0;

//-----速度参数-----//
__IO s16 g_BasePWM = 1000;		// 基本PWM(油门)-->起飞的最小PWM,第一版为1000左右
static u16 g_MaxPWM = 2000;		// 最大PWM-->运动过程中给出的最大PWM

//-----PID参数定义-----//
//+模式单轴P = 0.8时,D = 0.8~0.9
//x模式单轴P = 2.15时,D = 0.5~0.7
//20140111 P = 2.1,D = 0.4 ??
//20140112 P = 3.4,D = 0.75 回中极慢,不达标
//20140115 P = 2.5,D = 0.6 回中快，没有振荡.但是PD有静差(杆上)
//20140118 P = 3.2,D = 0.7 大角度回中会振荡两次,绳子上大致可以平衡
static float g_PID_Kp = 3.1f;			// PID控制比例系数
static float g_PID_Ki = 0.12f;			// PID控制积分系数
static float g_PID_Kd = 0.65f;			// PID控制微分系数
static float g_PID_Yaw_Kp = 8.0f;		// YAW单独的P参数

//-----遥控姿态给定-----//
__IO s16 g_Exp_Pitch = 0;	// 通过遥控给出的微调控制角度
__IO s16 g_Exp_Roll = 0;

//-----姿态误差结构体定义-----//
struct _Attitude_Error_Tag g_Attitude_Error = {0,0,0};

//-----电机PWM输出-----//
static s16 g_motor1_PWM = 0, g_motor2_PWM = 0, g_motor3_PWM = 0, g_motor4_PWM = 0;

/************************************************************************************************
函数名：void Quadrotor_Control(const float Exp_Pitch, const float Exp_Roll, const float Exp_Yaw)
说明：四旋翼控制函数，用于PWM计算和输出
入口：const float Exp_Pitch	期望倾仰角
			const float Exp_Roll	期望横滚角
			const float Exp_Yaw		期望偏航角
出口：无
备注：当前控制环为姿态控制环
其中有    大角度放弃控制    和    悬停黄灯指示
************************************************************************************************/
void Quadrotor_Control(const float Exp_Pitch, const float Exp_Roll, const float Exp_Yaw)
{
	s16 outputPWM_Pitch, outputPWM_Roll, outputPWM_Yaw;
	
	// --- 得到当前系统的误差-->利用期望角度减去当前角度
	g_Attitude_Error.g_Error_Pitch = Exp_Pitch - g_Pitch;
	g_Attitude_Error.g_Error_Roll = Exp_Roll - g_Roll;
	g_Attitude_Error.g_Error_Yaw = Exp_Yaw - g_Yaw;
	
	// --- 倾角太大，放弃控制
	if (fabs(g_Attitude_Error.g_Error_Pitch) >= 55 || fabs(g_Attitude_Error.g_Error_Roll) >= 55)
	{
		PWM2_LED = 0;			// 蓝灯亮起
		PWM_Set(0, 0, 0, 0);	// 停机
		return ;
	}
	PWM2_LED = 1;				// 蓝灯熄灭
	
	// --- 稳定指示灯,黄色.当角度值小于3°时，判定为基本稳定,黄灯亮起
	if (fabs(g_Attitude_Error.g_Error_Pitch) <= 3 && fabs(g_Attitude_Error.g_Error_Roll) <= 3)
		PWM4_LED = 0;
	else
		PWM4_LED = 1;
	
	// --- 积分运算与积分误差限幅
	if (fabs(g_Attitude_Error.g_Error_Pitch) <= 20)	// 积分分离-->在姿态误差角小于20°时引入积分
	{	// Pitch
		// 累加误差
		g_Attitude_Error.g_ErrorI_Pitch += g_Attitude_Error.g_Error_Pitch;

		// 积分限幅
		if (g_Attitude_Error.g_ErrorI_Pitch >= PITCH_I_MAX)
			g_Attitude_Error.g_ErrorI_Pitch = PITCH_I_MAX;
		else if (g_Attitude_Error.g_ErrorI_Pitch <= -PITCH_I_MAX)
			g_Attitude_Error.g_ErrorI_Pitch = -PITCH_I_MAX;	
	}
	if (fabs(g_Attitude_Error.g_Error_Roll) <= 20)
	{	// Roll
		// 累加误差		
		g_Attitude_Error.g_ErrorI_Roll += g_Attitude_Error.g_Error_Roll;
		
		// 积分限幅		
		if (g_Attitude_Error.g_ErrorI_Roll >= ROLL_I_MAX)
			g_Attitude_Error.g_ErrorI_Roll = ROLL_I_MAX;
		else if (g_Attitude_Error.g_ErrorI_Roll <= -ROLL_I_MAX)
			g_Attitude_Error.g_ErrorI_Roll = -ROLL_I_MAX;		
	}
	
	// --- PID运算-->这里的微分D运算并非传统意义上的利用前一次的误差减去上一次的误差得来
	// --- 而是直接利用陀螺仪的值来替代微分项,这样的处理非常好,因为巧妙利用了硬件设施,陀螺仪本身就是具有增量的效果
	outputPWM_Pitch = (s16)(g_PID_Kp * g_Attitude_Error.g_Error_Pitch + g_PID_Ki * g_Attitude_Error.g_ErrorI_Pitch - g_PID_Kd * g_MPU6050Data_Filter.gyro_x_c);
	outputPWM_Roll = (s16)(g_PID_Kp * g_Attitude_Error.g_Error_Roll + g_PID_Ki * g_Attitude_Error.g_ErrorI_Roll - g_PID_Kd * g_MPU6050Data_Filter.gyro_y_c);
	outputPWM_Yaw = (s16)(g_PID_Yaw_Kp * g_Attitude_Error.g_Error_Yaw);
		
	// --- 给出PWM控制量到四个电机-->X模式控制
	//特别注意，这里输出反相了，因为误差是反的
	g_motor1_PWM = g_BasePWM + outputPWM_Pitch + outputPWM_Roll + outputPWM_Yaw;
	g_motor2_PWM = g_BasePWM - outputPWM_Pitch + outputPWM_Roll - outputPWM_Yaw;
	g_motor3_PWM = g_BasePWM - outputPWM_Pitch - outputPWM_Roll + outputPWM_Yaw;
	g_motor4_PWM = g_BasePWM + outputPWM_Pitch - outputPWM_Roll - outputPWM_Yaw;
	// --- 去除偏航测试
//	g_motor1_PWM = g_BasePWM + outputPWM_Pitch + outputPWM_Roll;
//	g_motor2_PWM = g_BasePWM - outputPWM_Pitch + outputPWM_Roll;
//	g_motor3_PWM = g_BasePWM - outputPWM_Pitch - outputPWM_Roll;
//	g_motor4_PWM = g_BasePWM + outputPWM_Pitch - outputPWM_Roll;
	// --- 偏航单独测试
//	g_motor1_PWM = g_BasePWM + outputPWM_Yaw;
//	g_motor2_PWM = g_BasePWM - outputPWM_Yaw;
//	g_motor3_PWM = g_BasePWM + outputPWM_Yaw;
//	g_motor4_PWM = g_BasePWM - outputPWM_Yaw;
	// --- +模式单轴测试
//	g_motor2_PWM = g_BasePWM - outputPWM_Pitch;
//	g_motor4_PWM = g_BasePWM + outputPWM_Pitch;
	// --- x模式单轴测试
//	g_motor1_PWM = g_BasePWM + outputPWM_Pitch;
//	g_motor2_PWM = g_BasePWM - outputPWM_Pitch;
//	g_motor3_PWM = g_BasePWM - outputPWM_Pitch;
//	g_motor4_PWM = g_BasePWM + outputPWM_Pitch;
	// --- 力矩测试
//	g_motor1_PWM = g_BasePWM;
//	g_motor2_PWM = g_BasePWM;
//	g_motor3_PWM = g_BasePWM;
//	g_motor4_PWM = g_BasePWM;
	
	// --- PWM反向清零,因为没有反转
	if (g_motor1_PWM < 0)
		g_motor1_PWM = 0;
	if (g_motor2_PWM < 0)
		g_motor2_PWM = 0;
	if (g_motor3_PWM < 0)
		g_motor3_PWM = 0;
	if (g_motor4_PWM < 0)
		g_motor4_PWM = 0;
	
	// --- PWM限幅
	if (g_motor1_PWM >= g_MaxPWM)
		g_motor1_PWM = g_MaxPWM;
	if (g_motor2_PWM >= g_MaxPWM)
		g_motor2_PWM = g_MaxPWM;
	if (g_motor3_PWM >= g_MaxPWM)
		g_motor3_PWM = g_MaxPWM;
	if (g_motor4_PWM >= g_MaxPWM)
		g_motor4_PWM = g_MaxPWM;
		
	if (g_Fly_Enable)			// 允许起飞,给出PWM
		PWM_Set(g_motor1_PWM, g_motor2_PWM, g_motor3_PWM, g_motor4_PWM);
	else
		PWM_Set(0, 0, 0, 0);	// 停机
}
