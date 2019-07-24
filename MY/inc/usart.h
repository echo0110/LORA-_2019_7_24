#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"

void USART3_Int(u16 baud);
extern u8 receBuf[81];
extern u8 receCount;
extern  volatile uint8_t  Recevstate;
#endif 
/*----------------------德飞莱 技术论坛：www.doflye.net--------------------------*/
