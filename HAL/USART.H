#ifndef __USART_H__
#define __USART_H__

#include "stm32f10x.h"

//-----USART变量-----//
#define BAUD 115200

//-----发送和缓冲区Size设定宏定义-----//
#define TxBufferSize   10
#define RxBufferSize   TxBufferSize

extern __IO uint8_t TxCounter;				// 发送计数器
extern __IO uint8_t RxCounter;				// 接收计数器
extern uint8_t TxBuffer[TxBufferSize];		// 发送缓冲区
extern uint8_t RxBuffer[RxBufferSize];		// 接收缓冲区

//-----函数-----//
extern void Uart1_PutChar(const u8 ch);						// 串口1发送字符
extern void Uart1_PutString(const u8 * buf, const u8 len);	// 串口1发送字符串

#endif
