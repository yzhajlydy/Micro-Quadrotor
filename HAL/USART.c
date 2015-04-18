#include "stm32f10x.h"
#include "USART.H"

//-----发送和接收计数器-----//
//__IO uint8_t TxCounter = 0x00;
//__IO uint8_t RxCounter = 0x00;

//-----发送接收缓冲区-----//
//uint8_t TxBuffer[TxBufferSize];
//uint8_t RxBuffer[RxBufferSize];

/*************************************
函数名：void USART_Configuration(void)
说明：USART初始化
入口：无
出口：无
备注：无
*************************************/
void USART_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = BAUD;																				//波特率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;												//数据长度，如果有奇偶校验，则字长必须为9bits，参见stm32f10x_usart.h Line63
  USART_InitStructure.USART_StopBits = USART_StopBits_1;														//停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;																//奇偶校验
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		//硬件控制流
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;										//发送/接收模式
  USART_Init(USART1, &USART_InitStructure);																					//初始化USARTx
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); 																			//使能串口1的DMA发送 
  USART_Cmd(USART1, ENABLE);																												//使能USARTx
}

/*********************************
函数名：void Uartx_PutChar(u8 ch)
说明：串口1发送字符
入口：u8 ch  需要发送的字符
出口：无
备注：无
*********************************/
void Uart1_PutChar(const u8 ch)
{
   USART_SendData(USART1, (u8)ch);
   while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

/**********************************************
函数名：void Uartx_PutString(u8 * buf, u8 len)
说明：串口x发送字符串
入口：u8* buf  字符串数组
			u8 len 字符个数
出口：无
备注：无
*********************************************/
void Uart1_PutString(const u8 * buf, const u8 len)
{   
	u8 i;
	
	for(i = 0; i < len; i++)
		Uart1_PutChar(*buf++);
}
