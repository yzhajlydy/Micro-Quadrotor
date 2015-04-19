#include "stm32f10x.h"
#include "math.h"
#include "MPU6050.H"
#include "Fusion.H"

//-----陀螺仪标定系数-----//
#define GYRO_CALIBRATION_COFF 0.015267f		// 该系数和MPU6050中的量程增益对应

//-----姿态更新相关定义-----//
#define PI 3.1415926					// 圆周率
#define CNTLCYCLE 0.003f				// 控制周期
#define TWOKPDEF	(2.0f * 13.0f)		// 2倍Kp
#define TWOKIDEF	(2.0f * 0.008)		// 2倍Ki

//-----姿态角-----//
float g_Pitch = 0.0;	// 范围-180°~+180° -->上正下负
float g_Roll = 0.0;		// 范围-90°~+90°		-->左负右正
float g_Yaw = 0.0;		// 范围-180°~+180° -->逆时针为正,顺时针为负

//-----姿态相关参数-----//
__IO float g_twoKp = TWOKPDEF;													// 2倍Kp
__IO float g_twoKi = TWOKIDEF;													// 2倍Ki
__IO float g_q0 = 1.0f, g_q1 = 0.0f, g_q2 = 0.0f, g_q3 = 0.0f;					// 四元数
__IO float g_integralFBx = 0.0f, g_integralFBy = 0.0f, g_integralFBz = 0.0f;	// 载体坐标系上的积分误差

/***********************************************************
函数名：float invSqrt(float x)
说明：快速开方算法
入口：无
出口：无
备注：http://en.wikipedia.org/wiki/Fast_inverse_square_root
***********************************************************/
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
	
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
	
  return y;
}

/******************************************************
函数名：void ReadFromIMU(void)
说明：读取MPU6050的值，包括加速度计和陀螺仪，一共6个值
入口：无
出口：无
备注：无
******************************************************/
void ReadFromIMU(void)
{
	Get_Accel_Data(ACCEL_XOUT_H);		//加速度计
	Get_Gyro_Data(GYRO_XOUT_H);			//陀螺仪
}

/**************************************************
函数名：void IMU_Filter(void)
说明：IMU滤波，包括加速度计的滑动滤波和陀螺仪的标定
入口：无
出口：无
备注：采用窗口滑动滤波法，长度为ACC_FILTER_DELAY
用控制周期3ms*ACC_FILTER_DELAY得到滞后时间常数
属于一阶滞后的FIR滤波器，具体的之后环节
有待对加速度计采样观察后FFT查看频谱后给出
滞后的时间常数
**************************************************/
void IMU_Filter(void)
{
    s32 resultx = 0;
    static s32 s_resulttmpx[ACC_FILTER_DELAY] = {0};
    static u8 s_bufferCounterx = 0;
    static s32 s_totalx = 0;
		
	s32 resulty = 0;
    static s32 s_resulttmpy[ACC_FILTER_DELAY] = {0};
    static u8 s_bufferCountery = 0;
    static s32 s_totaly = 0;
		
	s32 resultz = 0;
    static s32 s_resulttmpz[ACC_FILTER_DELAY] = {0};
    static u8 s_bufferCounterz = 0;
    static s32 s_totalz = 0;

	// 加速度计滤波
    s_totalx -= s_resulttmpx[s_bufferCounterx];					// 从总和中删除头部元素的值，履行头部指针职责
    s_resulttmpx[s_bufferCounterx] = g_MPU6050Data.accel_x;		// 将采样值放到尾部指针处，履行尾部指针职责
    s_totalx += g_MPU6050Data.accel_x;		                    // 更新总和
                                                                   
    resultx = s_totalx / ACC_FILTER_DELAY;		                // 计算平均值，并输入到一个固定变量中
    s_bufferCounterx++;		                        			// 更新指针
    if (s_bufferCounterx == ACC_FILTER_DELAY)		            // 到达队列边界
        s_bufferCounterx = 0;
		g_MPU6050Data_Filter.accel_x_f = resultx;
				
    s_totaly -= s_resulttmpy[s_bufferCountery];
    s_resulttmpy[s_bufferCountery] = g_MPU6050Data.accel_y;
    s_totaly += g_MPU6050Data.accel_y;

    resulty = s_totaly / ACC_FILTER_DELAY;
    s_bufferCountery++;
    if (s_bufferCountery == ACC_FILTER_DELAY)
        s_bufferCountery = 0;
		g_MPU6050Data_Filter.accel_y_f = resulty;
		
    s_totalz -= s_resulttmpz[s_bufferCounterz];
    s_resulttmpz[s_bufferCounterz] = g_MPU6050Data.accel_z;
    s_totalz += g_MPU6050Data.accel_z;

    resultz = s_totalz / ACC_FILTER_DELAY;
    s_bufferCounterz++;
    if (s_bufferCounterz == ACC_FILTER_DELAY)
        s_bufferCounterz = 0;
		g_MPU6050Data_Filter.accel_z_f = resultz;
	
		// 陀螺仪标定-->减去初始时刻的值
		g_MPU6050Data_Filter.gyro_x_c = g_MPU6050Data.gyro_x - g_Gyro_xoffset;	// 减去标定获得的偏移
		g_MPU6050Data_Filter.gyro_y_c = g_MPU6050Data.gyro_y - g_Gyro_yoffset;
		g_MPU6050Data_Filter.gyro_z_c = g_MPU6050Data.gyro_z - g_Gyro_zoffset;
		
		// 这里可以手动标定,即让飞机水平旋转90度,找到一个参数使得Yaw刚好也是90度
		// 该参数也可以直接查看MPU6050的数据手册中给定的量程增益
		g_MPU6050Data_Filter.gyro_x_c *= GYRO_CALIBRATION_COFF;
		g_MPU6050Data_Filter.gyro_y_c *= GYRO_CALIBRATION_COFF;
		g_MPU6050Data_Filter.gyro_z_c *= GYRO_CALIBRATION_COFF;
}

/***************************************************************************************************
函数名：void IMUupdata(float gx, float gy, float gz, float ax, float ay, float az)
说明：IMU单元数据融合，更新姿态四元数
入口：float gx	陀螺仪x分量
			float gy	陀螺仪y分量
			float gz	陀螺仪z分量
			float ax	加速度计x分量
			float ay	加速度计y分量
			float az	加速度计z分量
出口：无
备注：核心思想:利用陀螺仪来计算高速动态下的姿态,利用加速度计来进行角度修正
加速度计测量的是竖直方向上的重力加速度在b系中的三个分量,这三个分量全部为精确值(参考值)
为了消除姿态解算误差,必须计算出另一个在b系中竖直方向上的向量
这两个向量做向量积,得到误差,并且消除误差,就是我们要达到的目的
我们发现,在n系中一个竖直方向上的向量[0 0 1],经过一个旋转矩阵之后可以转换到b系中
这个转换之后的向量当然是在b系中的竖直方向上的向量,这样我们就找到了这个向量
不难发现,那个旋转矩阵就是我们姿态解算的核心部分
当我们通过做向量积来修正这两个向量误差的时候,我们就间接的修改了旋转矩阵的值,于是就达到了我们的目的
这个旋转矩阵,可以是方向余弦矩阵,可以是四元数,可以是欧拉角
这里当然是四元数.
综上所述,我们通过两个坐标系(b系和n系)中同是竖直方向上的向量的误差修正的方式来间接的修正旋转矩阵
***************************************************************************************************/
void IMUupdata(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;				// 平方根的倒数
	float halfvx, halfvy, halfvz;	// 在当前载体坐标系中，重力分量在三个轴上的分量
	float halfex, halfey, halfez;	// 当前加速度计测得的重力加速度在三个轴上的分量与当前姿态在三个轴上的重力分量的误差,这里采用差积的方式
	float qa, qb, qc;
	
	gx = gx * PI / 180;	// 转换为弧度制
    gy = gy * PI / 180;
	gz = gz * PI / 180;
	
	// 如果加速度计处于自由落体状态，可能会出现这种情况，不进行姿态解算，因为会产生分母无穷大的情况
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
	{
		// 单位化加速度计,意义在于在变更了加速度计的量程之后不需要修改Kp参数,因为这里归一化了
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// 将当前姿态的重力在三个轴上的分量分离出来
		// 就是方向余弦旋转矩阵的第三列,注意是地理坐标系(n系)到载体坐标系(b系)的,不要弄反了.如果书上是b系到n系,转置即可
		// 始终记着你操纵的是飞机,所以一切的量都要以他的坐标系来做参考,所以坐标需要从地理坐标系上转换到载体坐标系上
		halfvx = g_q1 * g_q3 - g_q0 * g_q2;
		halfvy = g_q0 * g_q1 + g_q2 * g_q3;
		halfvz = g_q0 * g_q0 - 0.5f + g_q3 * g_q3;
	
		// 计算由当前姿态的重力在三个轴上的分量与加速度计测得的重力在三个轴上的分量的差,这里采用三维空间的差积(向量积)方法求差
		// 计算公式由矩阵运算推导而来 公式参见http://en.wikipedia.org/wiki/Cross_product 中的Mnemonic部分
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// 积分求误差,关于当前姿态分离出的重力分量与当前加速度计测得的重力分量的差值进行积分消除误差
		if(g_twoKi > 0.0f) 
		{
			g_integralFBx += g_twoKi * halfex * CNTLCYCLE;		// Ki积分
			g_integralFBy += g_twoKi * halfey * CNTLCYCLE;
			g_integralFBz += g_twoKi * halfez * CNTLCYCLE;
			gx += g_integralFBx;		// 将积分误差反馈到陀螺仪上，修正陀螺仪的值
			gy += g_integralFBy;
			gz += g_integralFBz;
		}
		else	// 不进行积分运算，只进行比例调节
		{
			g_integralFBx = 0.0f;
			g_integralFBy = 0.0f;
			g_integralFBz = 0.0f;
		}
		
		// 直接应用比例调节，修正陀螺仪的值
		gx += g_twoKp * halfex;
		gy += g_twoKp * halfey;
		gz += g_twoKp * halfez;
	}
	
	//以下为四元数微分方程.将陀螺仪和四元数结合起来，是姿态更新的核心算子
	//计算方法由矩阵运算推导而来
	//	.		1      
	//	q = - * q x Omega    式中左边是四元数的倒数,右边的x是四元数乘法,Omega是陀螺仪的值(即角速度)
	//			2
	//	 .
	//	[q0] 		[0		-wx		-wy		-wz]	[q0]
	//	 .				
	//	[q1]		[wx	  0		  wz		-wy]	[q1]
	//	 .	 =  											* 
	//	[q2]		[wy	 -wz	  0		  wx ]	[q2]
	//	 . 			
	//	[q3]		[wz 	wy	 -wx		0	 ]	[q3]
	gx *= (0.5f * CNTLCYCLE);
	gy *= (0.5f * CNTLCYCLE);
	gz *= (0.5f * CNTLCYCLE);
	qa = g_q0;
	qb = g_q1;
	qc = g_q2;
	g_q0 += (-qb * gx - qc * gy - g_q3 * gz);
	g_q1 += ( qa * gx + qc * gz - g_q3 * gy);
	g_q2 += ( qa * gy - qb * gz + g_q3 * gx);
	g_q3 += ( qa * gz + qb * gy -   qc * gx);
	
	// 单位化四元数,意义在于单位化四元数在空间旋转时是不会拉伸的,仅有旋转角度.这类似与线性代数里面的正交变换
	recipNorm = invSqrt(g_q0 * g_q0 + g_q1 * g_q1 + g_q2 * g_q2 + g_q3 * g_q3);
	g_q0 *= recipNorm;
	g_q1 *= recipNorm;
	g_q2 *= recipNorm;
	g_q3 *= recipNorm;
	
	// 四元数到欧拉角转换，转换顺序为Z-Y-X,参见<Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors>.pdf一文,P24
	// 注意此时的转换顺序是1-2-3，即X-Y-Z。但是由于画图方便，作者这里做了一个转换，即调换Z和X，所以顺序没变
	g_Yaw = atan2(2 * g_q1 * g_q2 + 2 * g_q0 * g_q3, g_q1 * g_q1 + g_q0 * g_q0 - g_q3 * g_q3 - g_q2 * g_q2) * 180 / PI;	// Yaw
	g_Roll = asin(-2 * g_q1 * g_q3 + 2 * g_q0* g_q2) * 180 / PI; 														// Roll
  g_Pitch = atan2(2 * g_q2 * g_q3 + 2 * g_q0 * g_q1, -2 * g_q1 * g_q1 - 2 * g_q2* g_q2 + 1) * 180 / PI; 				// Pitch
}
