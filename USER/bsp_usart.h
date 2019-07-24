#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
//#include "sys_config.h"

#define USART_REC_LEN  			300  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收


#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
#define DMA_Rec_Len 255
extern u8 DMA_Rece_Buf[DMA_Rec_Len];	   //DMA接收串口数据缓冲区
extern u16  Usart1_Rec_Cnt;             //本帧数据长度	
extern u8 receBuf[81];
extern u8 receCount;
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
extern volatile uint8_t  Recevstate;
//如果想串口中断接收，请不要注释以下宏定义
extern u8 Usart1_Over;//串口1接收一帧完成标志位
void uart_init(u32 bound);
void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx);//恢复DMA指针


#endif /* __USART1_H */
