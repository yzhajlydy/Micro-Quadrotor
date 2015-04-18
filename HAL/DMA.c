#include "stm32f10x.h"

//-----DMA传输缓冲区大小-----//
static u16 g_DMA1_mem_len = 0;

/***********************************************************************************************
函数名：void DMA_Configuration(DMA_Channel_TypeDef* DMA_CHx, u32 pbase, u32 mbase, u16 bufsize)
说明：DMA配置
入口：DMA_Channel_TypeDef* DMA_CHx	DMA通道
			u32 pbase											外围器件基址
			u32 mbase											内存基址
			u16 bufsize										缓冲区大小
出口：无
备注：无
************************************************************************************************/
void DMA_Configuration(DMA_Channel_TypeDef* DMA_CHx, u32 pbase, u32 mbase, u16 bufsize)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	DMA_DeInit(DMA_CHx);   																									//将DMA的通道x寄存器重设为缺省值
	g_DMA1_mem_len = bufsize;
	DMA_InitStructure.DMA_PeripheralBaseAddr = pbase;  											//DMA外设基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = mbase;  													//DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  										//数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = bufsize;  														//DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  			//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 								//内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外围器件数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 				//内存数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  													//工作在正常缓存模式，不循环
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 									//DMA通道x拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  													//DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, &DMA_InitStructure);  																//根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
}

/********************************************************
函数名：void DMA_StartTrans(DMA_Channel_TypeDef*DMA_CHx)
说明：DMA开始一次传输
入口：DMA_Channel_TypeDef* DMA_CHx	DMA通道
出口：无
备注：无
*********************************************************/
void DMA_StartTrans(DMA_Channel_TypeDef*DMA_CHx)
{
	DMA_Cmd(DMA_CHx, DISABLE);  																//关闭USART1 TX DMA1 所指示的通道
 	DMA_SetCurrDataCounter(DMA1_Channel4, g_DMA1_mem_len);			//DMA通道的DMA缓存的大小
 	DMA_Cmd(DMA_CHx, ENABLE);  																	//使能USART1 TX DMA1 所指示的通道
}
