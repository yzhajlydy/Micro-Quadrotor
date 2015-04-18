#ifndef __IIC_H__
#define __IIC_H__

#include "stm32f10x.h"

//-----IO方向设置-----//
#define SDA_IN()  {GPIOA->CRH&=0XFFF0FFFF;GPIOA->CRH|=8<<16;}	// 可以修改为输入模式,上下拉均可
#define SDA_OUT() {GPIOA->CRH&=0XFFF0FFFF;GPIOA->CRH|=3<<16;}	// 可以修改为50MHZ推挽输出，但是用寄存器的方式速度更快

//-----IO操作函数-----// 
#define IIC_SCL_H    GPIO_SetBits(GPIOA, GPIO_Pin_11) 			// SCL_H
#define IIC_SCL_L    GPIO_ResetBits(GPIOA, GPIO_Pin_11) 		// SCL_L
#define IIC_SDA_H    GPIO_SetBits(GPIOA, GPIO_Pin_12) 			// SDA_H
#define IIC_SDA_L    GPIO_ResetBits(GPIOA, GPIO_Pin_12) 		// SDA_L 
#define READ_SDA  	 GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12)  // 输入SDA


//-----IIC所有操作函数-----//
void IIC_Init(void);			// 初始化IIC的IO口				 
void IIC_Start(void);			// 发送IIC开始信号
void IIC_Stop(void);	  		// 发送IIC停止信号
void IIC_Send_Byte(u8 txd);		// IIC发送一个字节
u8 IIC_Read_Byte(u8 ack);		// IIC读取一个字节
u8 IIC_Wait_Ack(void); 			// IIC等待ACK信号
void IIC_Ack(void);				// IIC发送ACK信号
void IIC_NAck(void);			// IIC不发送ACK信号

#endif
