#include "stm32f10x.h"
#include "HAL.H"

//#define BOOTLOADER	// 需要修改工程中的ROM的SIZE

/*******************
	Designed by Nemo
*******************/

///////////////////////////////系统部分接口说明//////////////////////////////
/*****************************************************************************
//---------------------------四轴飞行器正面俯瞰图---------------------------//
-----------------------------------------------------------------------------
|					(Motor1)				 (Motor4)						|
|					x								x						|
|						x						x							|
|							x				x								|
|								x		x									|
|									x										|
|								x		x									|
|							x				x								|
|						x						x							|
|					x								x						|
|					(Motor2)				 (Motor3)						|
-----------------------------------------------------------------------------

					Motor1 && Motor3	-->		顺时针
					Motor2 && Motor4	-->		逆时针
					

LED1~4:		用来显示PWM信号的4个蓝色LED指示灯
				LED1-->PA0-->Motor1
				LED2-->PA1-->Motor2
				LED3-->PA2-->Motor3
				LED4-->PA3-->Motor4
PWM1~4_LED:	用来指示系统状态的四种不同颜色的指示灯
				PWM1_LED-->PB6-->白色
				PWM2_LED-->PB7-->蓝色
				PWM3_LED-->PB8-->翠绿
				PWM4_LED-->PB9-->黄色
PWM1~4:		用来驱动4个Motor的PWM信号
				PWM1-->PA6-->Motor1
				PWM2-->PA7-->Motor2
				PWM3-->PB0-->Motor3
				PWM4-->PB1-->Motor4
MPU_INT:	MPU6050中断输入口
				MOU_INT-->PA4
USART:		扩展串口
				USART1_TX-->PA9
				USART1_RX-->PA10
IIC:		IIC接口
				SCL-->PA11
				SDA-->PA12

NRF24L01接收的功能键映射:
"Up"			-->		247
"Down"			-->		251
"Left"			-->		253
"Right"			-->		254
"Start"			-->		239
"Select"		-->		223
"B"				-->		127
"A"				-->		191

*******************************************************************************
***		开机时有5秒钟时间接收bootloader更新，时间到后自动进入程序开始运行	***
***		如果接收到程序，黄灯闪烁两次。								   		***															
***		如果没有程序更新，黄灯不闪烁。										***
*******************************************************************************
//------------------------------故障信号指示--------------------------------//
LED:
		1、	同时500ms闪烁-->未检测到NRF24L01
		2、	亮起-->对应的Motor有PWM信号输出
PWM_LED:
		1、	PWM3_LED翠绿色每500ms闪烁一次表明系统正常运行
		2、	PWM4_LED黄色闪两次说明bootloader已经下载到程序并更新
			PWM4_LED黄灯亮起说明飞机的Pitch和Roll姿态误差在5°范围内
			熄灭说明飞机的角度比较大，会产生平移
		3、	PWM1_LED白色如果常亮说明程序执行一次的时间已经超过了控制周期
		4、	PWM2_LED蓝色亮起表明飞行器的Rool或者Pitch姿态已经严重偏离平衡状态
非常危险。

//----------------------------串口信号数据格式------------------------------//
0~11：加速度计*6，陀螺仪*6
12~15：Pitch*10
16~19：Roll*10
20~23：Yaw*10

*****************************************************************************/
//////////////////////////////////////////////////////////////////////////////

//-----外部变量声明-----//
extern u16 g_tim2counter;	// 定时器2计数器，用来在进入程序时的强制置0

//-----系统控制周期开关变量-----//
__IO u8 g_ConCycT_flag = 0;	// 每个控制周期开始时置1,volatile类型保证中断不出错

/*********************************************************
函数名：static void USARTData_Prepare(u8* pt2g_SendBuffer)
说明：USART数据发送准备，准备好后进行一次串口DMA
入口：u8* pt2g_SendBuffer	需要发送的串口缓冲区
出口：无
备注：无
*********************************************************/
static void USARTData_Prepare(u8* pt2g_SendBuffer)
{
	u8 i, j;
	s16 temp;
	struct MPU6050Filter_tag *ptF = &g_MPU6050Data_Filter;	// 指向加速度计滤波之后的结构体指针

	for (i = 0; i < 6; i++)		//加速度计数据准备
		for (j = 0; j < 2; j++)
			pt2g_SendBuffer[i * 2 + j] = (u8)(((*((s16*)ptF + i)) >> (j * 8)) & 0x00FF);	// 这里注意对结构体指针的理解，因为MPU6050结构体中全部为s16类型，所以不需要涉及到对齐问题
			
	temp = (s16)(g_Pitch * 10);
	pt2g_SendBuffer[12] = (temp >> 8) & 0x00FF;		//高8位
	pt2g_SendBuffer[13] = temp & 0x00FF;			//低8位
	temp = (s16)(g_Roll * 10);
	pt2g_SendBuffer[14] = (temp >> 8) & 0x00FF;		//高8位
	pt2g_SendBuffer[15] = temp & 0x00FF;			//低8位
	temp = (s16)(g_Yaw * 10);
	pt2g_SendBuffer[16] = (temp >> 8) & 0x00FF;		//高8位
	pt2g_SendBuffer[17] = temp & 0x00FF;			//低8位
}

/********
main函数
********/
int main(void)
{
	u8 convert_flag = 0;				// 该变量用来产生2倍于控制周期的时间,用来DMA的串口传送数据
	u8 g_SendBuffer[12 + 2*3] = {0};	// 串口发送缓冲区
	u8 old_NRFRevCnt = 0;				// 前一次NRF接收计数器
	u8 control_times = 0;				// 当遥控数据来之后的控制次数

#ifdef BOOTLOADER
  NVIC_SetVectorTable(FLASH_BASE, 0x8000);	// 重新设置中断向量表位置，Bootloader所需
#endif

	ChipHal_Init();		// 片内硬件初始化
	Delay_ms(500);		// 上电500ms系统传感器稳定
	ChipOutHal_Init();	// 片外硬件初始化
	Delay_ms(500);
	
	NRF24L01_RX_Mode();
	DMA_Configuration(DMA1_Channel4, (u32)&USART1->DR, (u32)g_SendBuffer, 12 + 2*3);	// 初始化DMA,通道4对应USART1
	IMU_Calibration();	// IMU标定
	g_tim2counter = 0;
	g_ConCycT_flag = 0;
	while(1)
	{
		if (g_ConCycT_flag)	// 3ms控制周期进来一次
		{
			g_ConCycT_flag = 0;
			convert_flag = !convert_flag;
			ReadFromIMU();		// 读取MPU6050的值
			IMU_Filter();		// 加速度计滤波与陀螺仪标定输出
			IMUupdata(g_MPU6050Data_Filter.gyro_x_c, g_MPU6050Data_Filter.gyro_y_c, g_MPU6050Data_Filter.gyro_z_c,	// 四元数姿态更新
								g_MPU6050Data_Filter.accel_x_f * 0.0098, g_MPU6050Data_Filter.accel_y_f * 0.0098, g_MPU6050Data_Filter.accel_z_f * 0.0098);	//*0.0098将g转换为mg
			if (convert_flag)	// 2*3=6ms发送一次
			{
				USARTData_Prepare(g_SendBuffer);	// 串口数据准备
				DMA_StartTrans(DMA1_Channel4);		// 开启一次DMA传输
			}
			if (g_NRFRevCnt != old_NRFRevCnt || control_times != 0)	// 有遥控控制量进来
			{
				PWM1_LED = 0;
				control_times++;
				if (control_times == 50)	// 控制50次,即150ms
					control_times = 0;
				Quadrotor_Control(g_Exp_Pitch,g_Exp_Roll,0);	// 计算PWM输出控制量-->(0,0,0)为悬停
			}
			else	// 悬停
			{
				PWM1_LED = 1;
				Quadrotor_Control(0,0,0);
			}
			old_NRFRevCnt = g_NRFRevCnt;	// 更新计数器
			if(g_ConCycT_flag)	// 超时，白灯亮起
				PWM1_LED = 0;
		}
	}
}
