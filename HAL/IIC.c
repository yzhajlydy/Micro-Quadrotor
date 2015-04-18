#include "stm32f10x.h"
#include "Delay.H"
#include "IIC.H"

/***************************
函数名：void IIC_Init(void)
说明：IIC初始化
入口：无
出口：无
备注：均输出高电平
****************************/
void IIC_Init(void)
{					     
	GPIO_SetBits(GPIOA, GPIO_Pin_11 | GPIO_Pin_12);
}

/***************************
函数名：void IIC_Start(void)
说明：产生IIC起始信号
入口：无
出口：无
备注：无
****************************/
void IIC_Start(void)
{
	SDA_OUT();     	//SDA线输出
	IIC_SDA_H;
	IIC_SCL_H;
	Delay_us(4);
 	IIC_SDA_L;
//	Delay_us(1);
	IIC_SCL_L;			//钳住I2C总线，准备发送或接收数据 
}

/***************************
函数名：void IIC_Stop(void)
说明：产生IIC停止信号
入口：无
出口：无
备注：无
****************************/
void IIC_Stop(void)
{
	SDA_OUT();			//SDA线输出
	IIC_SDA_L;
 	Delay_us(4);
	IIC_SCL_H;
//	Delay_us(1);
	IIC_SDA_H;			//发送I2C总线结束信号
	Delay_us(5);		//再次启动需要4.7us							   	
}

/*****************************
函数名：u8 IIC_Wait_Ack(void)
说明：等待应答信号到来
入口：无
出口：u8  1，接收应答失败
					0，接收应答成功
备注：无
*****************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime = 0;
	
	SDA_IN();      							//SDA设置为输入  
	IIC_SDA_H;
//	Delay_us(1);	   
	IIC_SCL_H;
//	Delay_us(1);	 
	while (READ_SDA)						//SDA为高，等待IIC器件拉低
	{
		ucErrTime++;
		if (ucErrTime > 250)			//40*250=1ms未答复，IIC发出停止信号
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L;
	return 0;
}

/*************************
函数名：void IIC_Ack(void)
说明：产生ACK应答
入口：无
出口：无
备注：无
**************************/
void IIC_Ack(void)
{
	IIC_SCL_L;
	SDA_OUT();
	IIC_SDA_L;
//	Delay_us(1);
	IIC_SCL_H;
//	Delay_us(1);
	IIC_SCL_L;
}

/**************************
函数名：void IIC_NAck(void)
说明：不产生ACK应答		
入口：无
出口：无
备注：无
***************************/
void IIC_NAck(void)
{
	IIC_SCL_L;
	SDA_OUT();
	IIC_SDA_H;
//	Delay_us(1);
	IIC_SCL_H;
//	Delay_us(1);
	IIC_SCL_L;
}

/*********************************
函数名：void IIC_Send_Byte(u8 txd)
说明：IIC发送一个字节
入口：u8 txd
出口：无
备注：无
*********************************/
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;  
	
		SDA_OUT(); 	    
    IIC_SCL_L;				//拉低时钟开始数据传输
    for (t = 0; t < 8; t++)
    {              
			if ((txd&0x80) >> 7)
				IIC_SDA_H;
			else
				IIC_SDA_L;
			txd <<= 1; 	  
//			Delay_us(1);
			IIC_SCL_H;
//			Delay_us(1); 
			IIC_SCL_L;
    }
		IIC_Wait_Ack();
}

/********************************
函数名：u8 IIC_Read_Byte(u8 ack)
说明：IIC读一个字节
入口：u8 ack 	ack=1，发送ACK
							ack=0，发送nACK
出口：u8 	1，有应答
					0，无应答		
备注：返回从机有无应答
********************************/
u8 IIC_Read_Byte(u8 ack)
{
	u8 i, receive = 0;
	
	SDA_IN();		//SDA设置为输入
  for (i = 0; i < 8; i++)
	{
		IIC_SCL_L; 
//		Delay_us(1);
		IIC_SCL_H;
		receive <<= 1;
		if (READ_SDA)				//收到1
			receive++;
//		Delay_us(1); 
  }					 
	if (!ack)
			IIC_NAck();				//发送nACK
	else
			IIC_Ack(); 				//发送ACK   
	return receive;
}
