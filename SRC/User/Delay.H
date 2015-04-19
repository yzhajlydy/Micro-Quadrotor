#ifndef __DELAY_H__
#define __DELAY_H__

#include "stm32f10x.h"

extern void Delay_Init(void);
extern void Delay_ms(u16 nms);
extern void Delay_us(u32 nus);

#endif
