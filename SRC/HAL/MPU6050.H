#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"

//-----MPU6050寄存器-----//
#define	SMPLRT_DIV			0x19	// 陀螺仪采样率,典型值:0x07(125Hz)
#define	MPU6050_CONFIG		0x1A	// 低通滤波频率,典型值:0x06(5Hz)
#define	GYRO_CONFIG			0x1B	// 陀螺仪自检及测量范围,典型值:0x18(不自检,2000deg/s)
#define	ACCEL_CONFIG		0x1C	// 加速度计自检、测量范围及高通滤波频率,典型值:0x00(不自检,2g,5Hz)
#define	ACCEL_XOUT_H		0x3B
#define	ACCEL_XOUT_L		0x3C
#define	ACCEL_YOUT_H		0x3D
#define	ACCEL_YOUT_L		0x3E
#define	ACCEL_ZOUT_H		0x3F
#define	ACCEL_ZOUT_L		0x40
#define	TEMP_OUT_H			0x41
#define	TEMP_OUT_L			0x42
#define	GYRO_XOUT_H			0x43
#define	GYRO_XOUT_L			0x44	
#define	GYRO_YOUT_H			0x45
#define	GYRO_YOUT_L			0x46
#define	GYRO_ZOUT_H			0x47
#define	GYRO_ZOUT_L			0x48
#define	PWR_MGMT_1			0x6B	// 电源管理,典型值:0x00(正常启用)

//-----MPU6050设备地址-----//
#define	MPU6050_DEVICE	0xD0		// IIC写入时的地址寄存器,+1为读取,AD0接地

//-----结构体变量声明-----//
struct MPU6050_tag	// IMU直接采样值
{
	s16 accel_x;	// 加速度计x
	s16 accel_y;	// 加速度计y
	s16 accel_z;	// 加速度计z
	s16 gyro_x;		// 陀螺仪x
	s16 gyro_y;		// 陀螺仪y
	s16 gyro_z;		// 陀螺仪z
};
struct MPU6050Filter_tag			// IMU滤波后的值
{
	s16 accel_x_f;	// 加速度计x滤波后的值
	s16 accel_y_f;	// 加速度计y滤波后的值
	s16 accel_z_f;	// 加速度计z滤波后的值
	s16 gyro_x_c;	// 陀螺仪标定后的值
	s16 gyro_y_c;	// 陀螺仪标定后的值
	s16 gyro_z_c;	// 陀螺仪标定后的值
};

//-----变量声明-----//
extern struct MPU6050_tag g_MPU6050Data;				// IMU直接采样值
extern struct MPU6050Filter_tag g_MPU6050Data_Filter;	// IMU滤波后的结构体
extern s32 g_Gyro_xoffset;								// IMU标定后的误差
extern s32 g_Gyro_yoffset;
extern s32 g_Gyro_zoffset;

//-----函数声明-----//
extern void MPU6050_Init(void);							// 初始化MPU6050
extern void MPU6050_WirteByte(u8 reg, u8 data);			// MPU6050单次写字节
extern u8 MPU6050_ReadByte(u8 reg);						// MPU6050单次读字节
extern void	Get_Accel_Data(u8 reg);						// 得到MPU6050加速度计数据
extern void	Get_Gyro_Data(u8 reg);						// 得到MPU6050陀螺仪数据
extern void IMU_Calibration(void);						// MPU6050标定

#endif
