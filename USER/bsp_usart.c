/**
  ******************************************************************************
  * @file    bsp_usart1.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   usart应用bsp
  ******************************************************************************
  * @attention
  *
  * 实验平台:野火 iSO-MINI STM32 开发板 
  * 论坛    :http://www.chuxue123.com
  * 淘宝    :http://firestm32.taobao.com
  *
  ******************************************************************************
  */
#include <stdio.h>
#include "stm32f10x.h" 
#include "bsp_usart.h"
#include  "delay.h"

////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/8/18
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  
u8 receBuf[81];//串口4接收数组
u8  DMA_Rece_Buf[DMA_Rec_Len];	   //DMA接收串口数据缓冲区
u16  Usart1_Rec_Cnt=0;             //本帧数据长度	
u8 Usart1_Over=0;//串口1接收一帧完成标志位

volatile uint8_t  Recevstate=0;//接收一帧数据标志
u8 receCount;  //单片机串口接收到的字节数

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*使用microLib的方法*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  
  
void uart_init(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输
	 
	USART_DeInit(USART1);  //复位串口1
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//
  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  /*USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 */
	
	USART_Init(USART1, &USART_InitStructure); //初始化串口
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//开启空闲中断
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);   //使能串口1 DMA接收
	USART_Cmd(USART1, ENABLE);                    //使能串口 
	
	//相应的DMA配置
		DMA_DeInit(DMA1_Channel5);   //将DMA的通道5寄存器重设为缺省值  串口1对应的是DMA通道5
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;  //DMA外设ADC基地址
		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)DMA_Rece_Buf;  //DMA内存基地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;  //数据传输方向，从外设读取发送到内存
		DMA_InitStructure.DMA_BufferSize = DMA_Rec_Len;  //DMA通道的DMA缓存的大小
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
		DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级 
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
		DMA_Init(DMA1_Channel5, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
				
    DMA_Cmd(DMA1_Channel5, ENABLE);  //正式驱动DMA传输
	
}
/*重新恢复DMA指针*/
  	void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx)
	{ 
		DMA_Cmd(DMA_CHx, DISABLE );  //关闭USART1 TX DMA1 所指示的通道      
		DMA_SetCurrDataCounter(DMA_CHx,DMA_Rec_Len);//DMA通道的DMA缓存的大小
		DMA_Cmd(DMA_CHx, ENABLE);  //使能USART1 TX DMA1 所指示的通道 
	} 

void USART_putchar(USART_TypeDef* USART_x, unsigned char ch)
{
/* Write a character to the USART */
    USART_SendData(USART_x, (unsigned char) ch);
    while( USART_GetFlagStatus(USART_x, USART_FLAG_TXE) == RESET );
    while( USART_GetFlagStatus(USART_x, USART_FLAG_TC) == RESET );
}

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
		//delay_ms(0);
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
		uint8_t clear=clear;//消除编译器没有用过的警告
	 if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)   //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		  //receBuf[receCount++]=USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		  USART_ReceiveData(USART1);//读取数据 注意：这句必须要，否则不能够清除中断标志位。我也不知道为啥！
		  Usart1_Rec_Cnt = DMA_Rec_Len-DMA_GetCurrDataCounter(DMA1_Channel5);	//算出接本帧数据长度
			Usart1_Over = 1;//标志接收到一帧数据
			USART_ClearITPendingBit(USART1, USART_IT_IDLE);         //清除中断标志
			MYDMA_Enable(DMA1_Channel5);                   //恢复DMA指针，等待下一次的接收
		} 
//	 else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  //如果接收到了1帧数据
//	  {
//	    clear=USART1->SR;
//		  clear=USART1->DR;//先读SR 再读DR清IDLE中断
//	   	Recevstate=1;//接收到了一帧数据
//	  }
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 


#endif	

