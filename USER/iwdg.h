#ifndef __WDG_H
#define __WDG_H
//#include "sys.h"
#include "stm32f10x.h" 


extern void IWDG_Init(u8 prer,u16 rlr);
extern void IWDG_Feed(void);

 
#endif
