/*-------------------------------------------------------------------------------
文件名称：usart.c
文件描述：设置串口参数
硬件平台：尼莫M3S开发板
编写整理：shifang
固件库  ：V3.5
技术论坛：www.doflye.net
备    注：通过简单修改可以移植到其他开发板，部分资料来源于网络。
---------------------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f10x.h"
#include "usart.h"

u8 receBuf[81];//串口3接收数组
volatile uint8_t  Recevstate=0;//接收一帧数据标志
u8 receCount;  //串口接收数据字节数

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void USART3_Int(u16 baud)
{

    GPIO_InitTypeDef GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //使能UART3所在的GPIOB的时钟
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);//串口时钟配置
 	  USART_DeInit(USART3);  //复位串口1
	  //USART3_TX   PB.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化PB10
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);  //初始化PB11

   //Usart1 NVIC 配置

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
 
  /* USARTx configured as follow:
        - BaudRate = 9600 baud  波特率
        - Word Length = 8 Bits  数据长度
        - One Stop Bit          停止位
        - No parity             校验方式
        - Hardware flow control disabled (RTS and CTS signals) 硬件控制流
        - Receive and transmit enabled                         使能发送和接收
  */
  USART_InitStructure.USART_BaudRate = baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启中断
  USART_Cmd(USART3, ENABLE);                    //使能串口 
}


PUTCHAR_PROTOTYPE
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART */
  USART_SendData(USART3, (uint8_t) ch);

  /* 循环等待直到发送结束*/
  while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
  {}

  return ch;
}
//重定义函数
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
}*/


/*
函数名称：USART3_IRQHandler(void) 
功能    :idm30 串口数据接收
参数    ：无
返回值  ：无
autor   : niub 
*/
void USART3_IRQHandler(void)                	//纯�1中断服务程序
	{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
		uint8_t clear=clear;//消除编译器没有用过的警告
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		 receBuf[receCount++]=USART_ReceiveData(USART3);//(USART1->DR);	//读取接收到的数据
		}
    else if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)  //如果接收到了1帧数据
	  {
	    clear=USART3->SR;
		  clear=USART3->DR;//先读SR 再读DR清IDLE中断
	   	Recevstate=1;//接收到了一帧数据
	  }

 
		
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 



/*----------------------德飞莱 技术论坛：www.doflye.net--------------------------*/
