/*-------------------------------------------------------------------------------
�ļ����ƣ�usart.c
�ļ����������ô��ڲ���
Ӳ��ƽ̨����ĪM3S������
��д����shifang
�̼���  ��V3.5
������̳��www.doflye.net
��    ע��ͨ�����޸Ŀ�����ֲ�����������壬����������Դ�����硣
---------------------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f10x.h"
#include "usart.h"

u8 receBuf[81];//����3��������
volatile uint8_t  Recevstate=0;//����һ֡���ݱ�־
u8 receCount;  //���ڽ��������ֽ���

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
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //ʹ��UART3���ڵ�GPIOB��ʱ��
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);//����ʱ������
 	  USART_DeInit(USART3);  //��λ����1
	  //USART3_TX   PB.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ��PB10
   
    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);  //��ʼ��PB11

   //Usart1 NVIC ����

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
 
  /* USARTx configured as follow:
        - BaudRate = 9600 baud  ������
        - Word Length = 8 Bits  ���ݳ���
        - One Stop Bit          ֹͣλ
        - No parity             У�鷽ʽ
        - Hardware flow control disabled (RTS and CTS signals) Ӳ��������
        - Receive and transmit enabled                         ʹ�ܷ��ͺͽ���
  */
  USART_InitStructure.USART_BaudRate = baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART3, &USART_InitStructure);
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�����ж�
  USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ��� 
}


PUTCHAR_PROTOTYPE
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART */
  USART_SendData(USART3, (uint8_t) ch);

  /* ѭ���ȴ�ֱ�����ͽ���*/
  while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
  {}

  return ch;
}
//�ض��庯��
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
�������ƣ�USART3_IRQHandler(void) 
����    :idm30 �������ݽ���
����    ����
����ֵ  ����
autor   : niub 
*/
void USART3_IRQHandler(void)                	//���1�жϷ������
	{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
		uint8_t clear=clear;//����������û���ù��ľ���
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
		 receBuf[receCount++]=USART_ReceiveData(USART3);//(USART1->DR);	//��ȡ���յ�������
		}
    else if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)  //������յ���1֡����
	  {
	    clear=USART3->SR;
		  clear=USART3->DR;//�ȶ�SR �ٶ�DR��IDLE�ж�
	   	Recevstate=1;//���յ���һ֡����
	  }

 
		
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();  											 
#endif
} 



/*----------------------�·��� ������̳��www.doflye.net--------------------------*/
