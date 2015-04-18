#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f10x.h"

void SPI_Configuration(void);		// 初始化SPI口
u8 SPI_ReadWriteByte(u8 txData);	// SPI总线读写一个字节

#endif
