/*-------------------------------------------------------------------------------
ÎÄ¼þÃû³Æ£ºusart.c
ÎÄ¼þÃèÊö£ºÉèÖÃ´®¿Ú²ÎÊý
Ó²¼þÆ½Ì¨£ºÄáÄªM3S¿ª·¢°å
±àÐ´ÕûÀí£ºshifang
¹Ì¼þ¿â  £ºV3.5
¼¼ÊõÂÛÌ³£ºwww.doflye.net
±¸    ×¢£ºÍ¨¹ý¼òµ¥ÐÞ¸Ä¿ÉÒÔÒÆÖ²µ½ÆäËû¿ª·¢°å£¬²¿·Ö×ÊÁÏÀ´Ô´ÓÚÍøÂç¡£
---------------------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f10x.h"
#include "usart.h"

u8 receBuf[81];//´®¿Ú3½ÓÊÕÊý×é
volatile uint8_t  Recevstate=0;//½ÓÊÕÒ»Ö¡Êý¾Ý±êÖ¾
u8 receCount;  //´®¿Ú½ÓÊÕÊý¾Ý×Ö½ÚÊý

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
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //Ê¹ÄÜUART3ËùÔÚµÄGPIOBµÄÊ±ÖÓ
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);//´®¿ÚÊ±ÖÓÅäÖÃ
 	  USART_DeInit(USART3);  //¸´Î»´®¿Ú1
	  //USART3_TX   PB.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//¸´ÓÃÍÆÍìÊä³ö
    GPIO_Init(GPIOB, &GPIO_InitStructure); //³õÊ¼»¯PB10
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//¸¡¿ÕÊäÈë
    GPIO_Init(GPIOB, &GPIO_InitStructure);  //³õÊ¼»¯PB11

   //Usart1 NVIC ÅäÖÃ

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//ÇÀÕ¼ÓÅÏÈ¼¶3
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//×ÓÓÅÏÈ¼¶3
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQÍ¨µÀÊ¹ÄÜ
	  NVIC_Init(&NVIC_InitStructure);	//¸ù¾ÝÖ¸¶¨µÄ²ÎÊý³õÊ¼»¯VIC¼Ä´æÆ÷
 
  /* USARTx configured as follow:
        - BaudRate = 9600 baud  ²¨ÌØÂÊ
        - Word Length = 8 Bits  Êý¾Ý³¤¶È
        - One Stop Bit          Í£Ö¹Î»
        - No parity             Ð£Ñé·½Ê½
        - Hardware flow control disabled (RTS and CTS signals) Ó²¼þ¿ØÖÆÁ÷
        - Receive and transmit enabled                         Ê¹ÄÜ·¢ËÍºÍ½ÓÊÕ
  */
  USART_InitStructure.USART_BaudRate = baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//¿ªÆôÖÐ¶Ï
  USART_Cmd(USART3, ENABLE);                    //Ê¹ÄÜ´®¿Ú 
}


PUTCHAR_PROTOTYPE
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART */
  USART_SendData(USART3, (uint8_t) ch);

  /* Ñ­»·µÈ´ýÖ±µ½·¢ËÍ½áÊø*/
  while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
  {}

  return ch;
}
//ÖØ¶¨Òåº¯Êý
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
º¯ÊýÃû³Æ£ºUSART3_IRQHandler(void) 
¹¦ÄÜ    :idm30 ´®¿ÚÊý¾Ý½ÓÊÕ
²ÎÊý    £ºÎÞ
·µ»ØÖµ  £ºÎÞ
autor   : niub 
*/
void USART3_IRQHandler(void)                	//´¿Ú1ÖÐ¶Ï·þÎñ³ÌÐò
	{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//Èç¹ûSYSTEM_SUPPORT_OSÎªÕæ£¬ÔòÐèÒªÖ§³ÖOS.
	OSIntEnter();    
#endif
		uint8_t clear=clear;//Ïû³ý±àÒëÆ÷Ã»ÓÐÓÃ¹ýµÄ¾¯¸æ
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //½ÓÊÕÖÐ¶Ï(½ÓÊÕµ½µÄÊý¾Ý±ØÐëÊÇ0x0d 0x0a½áÎ²)
		{
		 receBuf[receCount++]=USART_ReceiveData(USART3);//(USART1->DR);	//¶ÁÈ¡½ÓÊÕµ½µÄÊý¾Ý
		}
    else if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)  //Èç¹û½ÓÊÕµ½ÁË1Ö¡Êý¾Ý
	  {
	    clear=USART3->SR;
		  clear=USART3->DR;//ÏÈ¶ÁSR ÔÙ¶ÁDRÇåIDLEÖÐ¶Ï
	   	Recevstate=1;//½ÓÊÕµ½ÁËÒ»Ö¡Êý¾Ý
	  }

 
		
#if SYSTEM_SUPPORT_OS 	//Èç¹ûSYSTEM_SUPPORT_OSÎªÕæ£¬ÔòÐèÒªÖ§³ÖOS.
	OSIntExit();  											 
#endif
} 



/*----------------------µÂ·ÉÀ³ ¼¼ÊõÂÛÌ³£ºwww.doflye.net--------------------------*/
