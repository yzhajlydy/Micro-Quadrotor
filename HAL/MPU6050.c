#include "stm32f10x.h"
#include "Delay.h"
#include "IIC.H"
#include "MPU6050.H"
#include "Fusion.H"

//-----IMU直接采样值结构体-----//
struct MPU6050_tag g_MPU6050Data;
//-----IMU滤波、标定后的结构体-----//
struct MPU6050Filter_tag g_MPU6050Data_Filter;
//-----IMU标定参数-----//
s32 g_Gyro_xoffset = 0, g_Gyro_yoffset = 0, g_Gyro_zoffset = 0;
s32 g_Acc_xoffset = 0, g_Acc_yoffset = 0, g_Acc_zoffset = 0;

/*******************************
函数名：void MPU6050_Init(void)
说明：MPU6050初始化
入口：无
出口：无
备注：无
********************************/
void MPU6050_Init(void)
{
	MPU6050_WirteByte(PWR_MGMT_1, 0x00);			// 解除休眠状态,使能温度传感器
	MPU6050_WirteByte(SMPLRT_DIV, 0x01);			// 陀螺仪和加速度计采样率，与陀螺仪输出频率相关，又与DLPF相关。带宽越大，越灵敏，噪声越大，需要的输出频率越大，从而采样率越大
	MPU6050_WirteByte(MPU6050_CONFIG, 0x06);		// DLPF中的带宽设置为最小5Hz，虽然不灵敏，但是噪声小
	MPU6050_WirteByte(GYRO_CONFIG, 0x08);			// 量程为500°/s,(SMPLRT_DIV + 1) / 1KHz= 500Hz采样频率，陀螺仪输出频率为1KHz
	MPU6050_WirteByte(ACCEL_CONFIG, 0x08);			// 量程为+/-4g
}

/***********************************************
函数名：void MPU6050_WirteByte(u8 reg, u8 data)
说明：MPU6050单次写字节
入口：u8 reg		寄存器地址
			u8 data		需要写入的字节
出口：无
备注：无
***********************************************/
void MPU6050_WirteByte(u8 reg, u8 data)
{
	IIC_Start();
	IIC_Send_Byte(MPU6050_DEVICE);		// 发送器件地址+写信号
	IIC_Send_Byte(reg);					// 寄存器地址
	IIC_Send_Byte(data);				// 需要写入的数据
	IIC_Stop();
}

/***********************************
函数名：u8 MPU6050_ReadByte(u8 reg)
说明：MPU6050单次读字节
入口：u8	reg		寄存器地址
出口：u8	读取到的字节
备注：无
************************************/
u8 MPU6050_ReadByte(u8 reg)
{
	u8 tmp;
	
	IIC_Start();
	IIC_Send_Byte(MPU6050_DEVICE);			// 发送器件地址+写信号
	IIC_Send_Byte(reg);						// 需要读取的寄存器地址
	IIC_Start();
	IIC_Send_Byte(MPU6050_DEVICE + 1);		// 发送器件地址+读信号
	tmp = IIC_Read_Byte(0);					// 读取数据不产生应答
	IIC_Stop();
	
	return tmp;
}

/***********************************
函数名：void Get_Accel_Data(u8 reg)
说明：得到MPU6050加速度计数据
入口：u8	reg		寄存器地址
出口：无
备注：无
***********************************/
void Get_Accel_Data(u8 reg)
{
	u8 H1, L1;
	u8 H2, L2;
	u8 H3, L3;
	
	IIC_Start();
	IIC_Send_Byte(MPU6050_DEVICE);			// 发送器件地址+写信号
	IIC_Send_Byte(reg);						// 需要读取的寄存器地址
	IIC_Start();
	IIC_Send_Byte(MPU6050_DEVICE + 1);		// 发送器件地址+读信号
	H1 = IIC_Read_Byte(1);					// 读取数据产生应答
	L1 = IIC_Read_Byte(1);					// 按照地址顺序从低往高读取
	H2 = IIC_Read_Byte(1);
	L2 = IIC_Read_Byte(1);
	H3 = IIC_Read_Byte(1);
	L3 = IIC_Read_Byte(0);
	IIC_Stop();
	
	g_MPU6050Data.accel_x = (H1 << 8) + L1;
	g_MPU6050Data.accel_y = (H2 << 8) + L2;
	g_MPU6050Data.accel_z = (H3 << 8) + L3;
}

/**********************************
函数名：void Get_Gyro_Data(u8 reg)
说明：得到MPU6050陀螺仪数据
入口：u8	reg		寄存器地址
出口：无
备注：无
**********************************/
void Get_Gyro_Data(u8 reg)
{
	u8 H1, L1;
	u8 H2, L2;
	u8 H3, L3;
	
	IIC_Start();
	IIC_Send_Byte(MPU6050_DEVICE);				// 发送器件地址+写信号
	IIC_Send_Byte(reg);							// 需要读取的寄存器地址
	IIC_Start();
	IIC_Send_Byte(MPU6050_DEVICE + 1);			// 发送器件地址+读信号
	H1 = IIC_Read_Byte(1);						// 读取数据产生应答
	L1 = IIC_Read_Byte(1);						// 按照地址顺序从低往高读取
	H2 = IIC_Read_Byte(1);
	L2 = IIC_Read_Byte(1);
	H3 = IIC_Read_Byte(1);
	L3 = IIC_Read_Byte(0);
	IIC_Stop();

	g_MPU6050Data.gyro_x = (H1 << 8) + L1;
	g_MPU6050Data.gyro_y = (H2 << 8) + L2;
	g_MPU6050Data.gyro_z = (H3 << 8) + L3;
}

/**********************************
函数名：void IMU_Calibration(void)
说明：MPU6050标定
入口：无
出口：无
备注：用来开机时设定陀螺仪的零值
**********************************/
void IMU_Calibration(void)
{
	u8 i;

	for (i = 0; i < 30; i++)	//连续采样30次，一共耗时30*3=90ms
	{
		ReadFromIMU();			// 读取MPU6050的值
		g_Gyro_xoffset += g_MPU6050Data.gyro_x;
		g_Gyro_yoffset += g_MPU6050Data.gyro_y;
		g_Gyro_zoffset += g_MPU6050Data.gyro_z;
		Delay_ms(3);
	}
	g_Gyro_xoffset /= 30;		// 得到标定偏移
	g_Gyro_yoffset /= 30;
	g_Gyro_zoffset /= 30;
}
