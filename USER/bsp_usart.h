#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
//#include "sys_config.h"

#define USART_REC_LEN  			300  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����


#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
#define DMA_Rec_Len 255
extern u8 DMA_Rece_Buf[DMA_Rec_Len];	   //DMA���մ������ݻ�����
extern u16  Usart1_Rec_Cnt;             //��֡���ݳ���	
extern u8 receBuf[81];
extern u8 receCount;
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
extern volatile uint8_t  Recevstate;
//����봮���жϽ��գ��벻Ҫע�����º궨��
extern u8 Usart1_Over;//����1����һ֡��ɱ�־λ
void uart_init(u32 bound);
void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx);//�ָ�DMAָ��


#endif /* __USART1_H */
