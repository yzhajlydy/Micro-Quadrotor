#ifndef __DMA_H__
#define __DMA_H__

#include "stm32f10x.h"

extern void DMA_Configuration(DMA_Channel_TypeDef* DMA_CHx, u32 pbase, u32 mbase, u16 bufsize);	// DMA配置
extern void DMA_StartTrans(DMA_Channel_TypeDef* DMA_CHx);										// DMA开始一次传输

#endif
