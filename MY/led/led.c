/*-------------------------------------------------------------------------------
�ļ����ƣ�led.c
�ļ�����������Ӳ����������LED�˿ڣ��򿪶�Ӧ�ļĴ���        
��    ע����
---------------------------------------------------------------------------------*/
#include  "stdio.h"  
#include "string.h"
#include "stm32f10x.h"
#include "SX1276.h"
#include "led.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include  "delay.h"
#include "timer.h"
#include "led.h"
#include  "usart.h"
#include  "103.h" 
#include  "spi.h"
#include  "key.h"
#include "config.h"





 u8  temp_buf2[255];//�����߽�������
unsigned char  RF_EX0_STATUS; //RF
unsigned char  x4,x5; //RF
unsigned char   CRC_Value;
unsigned char recv[255];
unsigned char   SX1278_RLEN;
unsigned char t;
volatile unsigned char timer;
int j=0;
/*-------------------------------------------------------------------------------
�������ƣ�LED_Init
������������ʼ��LED��ض˿ڣ��򿪶˿�ʱ�ӣ����ö˿����  
�����������
���ز�������
��    ע����
---------------------------------------------------------------------------------*/
void LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //��PA��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//PB5,PE5��������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	//�˿��ٶ�
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//�˿�ģʽ����Ϊ�������ģʽ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//��ʼ����Ӧ�Ķ˿�
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
}

void Timer2_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��	 
	TIM_TimeBaseStructure.TIM_Period = arr;//��װֵ //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc;//��Ƶֵ //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //������ֵд����Ӧ�ļĴ���
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);  //ʹ�ܻ���ʧ��ָ����TIM�ж�
		

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //������ֵд����Ӧ�ļĴ���
 	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx����							 
}

//ͨ�ö�ʱ���жϳ�ʼ��	ͨ�ö�ʱ�� 2��3��4
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��2,3!
void Timer3_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��	  
//	TIM_TimeBaseStructure.TIM_Period = 5000; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
//	TIM_TimeBaseStructure.TIM_Prescaler =(7200-1); //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
//	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
//		TIM3, //TIM3
//		TIM_IT_Update  |  //TIM �ж�Դ
//		TIM_IT_Trigger,   //TIM �����ж�Դ 
//		ENABLE  //ʹ��
//		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_ClearFlag(TIM3, TIM_FLAG_Update); //�������жϱ�־

//	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx����
}

void Timer3_enable( void )
{
	TIM_ClearFlag(TIM3, TIM_FLAG_Update); //�������жϱ�־
	TIM_SetCounter(TIM3,0x00);			//���������ֵ
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
}

void Timer3_disable (void)
{
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE); //ʱ��ʧ��
//	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
	TIM_ITConfig(TIM3, TIM_IT_Update | TIM_IT_Trigger,DISABLE );
	TIM_Cmd(TIM3, DISABLE);  //ʧ��TIMx����
}


/*-------------------------------------------------------------------------------
�������ƣ�TIM2_IRQHandler(void)
����������TIM2�ж�
�����������
���ز�������
��    ע����
*/
void TIM2_IRQHandler(void)   //TIM2�ж�
{ 	
  static int Count=0; 	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //���ָ�����жϷ������
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMX���жϴ���λ	 
			
		 KeyRead(); 
     KeyProc();
     if(j==1)
		 {
			 Count++;
			if(Count==10)
		 	{
				LED4_REV;
				Count=0;
			}
     }					
	 //next:led_toggle();		 
		//LED4_REV;
		
		//timer++; // 1ms�ж�һ��,
		
	//	TIM2->CR1&=~TIM_CR1_CEN;//�رն�ʱ��
	}
}

/*-------------------------------------------------------------------------------
�������ƣ�TIM3_IRQHandler(void)
����������TIM3�ж�
�����������
���ز�������
��    ע����
*/
void TIM3_IRQHandler(void)   //TIM3�ж�
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
		//Timer3_disable();
	
		Timer3_enable();
		 
	}
}
/*-------------------------------------------------------------------------------
�������ƣ�void led_toggle();
����������LED��ƽ��ת
�����������
���ز�������
��    ע����
*/
void led_toggle(void)
{
	LED4_REV;		
}




 void SetTxPacket(void)
{  
		  
		    RF_EX0_STATUS=SX1276ReadData(REG_LR_IRQFLAGS);		  
		   if((RF_EX0_STATUS&0x40)==0x40)  //�������   
		 {
  		 CRC_Value=SX1276ReadData(REG_LR_MODEMCONFIG2);
			if((CRC_Value&0x04)==0x04)  //crc_value=b7
			{
	 		 SX1276WriteData(REG_LR_FIFOADDRPTR,0x00);      //??SPI???FIFO?????????
			 SX1278_RLEN = SX1276ReadData(REG_LR_NBRXBYTES); //??????????
			 switch_cs(enOpen);
		 	 SPI1_ReadWriteByte(0x00);//
			for(t=0;t<SX1278_RLEN;t++)
				{
				  recv[t]=SPI1_ReadWriteByte(0xff);	
				}
				 switch_cs(enClose);
			{
				 LED_TX_OFF;//����������ָʾLED-RX
		    UART_Send(recv,SX1278_RLEN);//������  ͨ�����ڷ���ȥ
				SX1278_RLEN=0;//����
				LED_TX_ON;//�ر�LED-RX
			}
			SX1276LoRaSetOpMode(stdby_mode);
	    SX1276WriteData(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);    
		  SX1276WriteData(REG_LR_HOPPERIOD, PACKET_MIAX_Value);//0x24  
		  SX1276WriteData( REG_LR_DIOMAPPING1, 0X00 ); // ? ?  ? ?I  
		  SX1276WriteData( REG_LR_DIOMAPPING2, 0x00 );	
		  SX1276LoRaSetOpMode(Recevived_mode);
		}
			
    }
		 else if((RF_EX0_STATUS&0x08)==0x08)  //�������
	  {//������ɺ�,��Ϊ����ģʽ
		 // SX1276LoRaSetOpMode(stdby_mode );
		 SX1276WriteData(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);	        //??????
		 SX1276WriteData(REG_LR_HOPPERIOD,   PACKET_MIAX_Value );      //0x24???,???????????
		 SX1276WriteData( REG_LR_DIOMAPPING1, 0X00 );
		 SX1276WriteData( REG_LR_DIOMAPPING2, 0x00 );	
		 SX1276LoRaSetOpMode(Recevived_mode);
		 SX1276WriteData( REG_LR_FIFOTXBASEADDR, 0);           //?Tx FIFO??
	   SX1276WriteData( REG_LR_FIFOADDRPTR, 0 ); 
		
	  }
		else if((RF_EX0_STATUS&0x04)==0x04)  //cad���  (���Ӧ������һ�ֽ��շ�ʽ)
	 {  
		if((RF_EX0_STATUS&0x01)==0x01)
		{	 
		//��ʾCAD��⵽��Ƶ�ź� ģ��������ģʽ
			SX1276LoRaSetOpMode( stdby_mode );
			SX1276WriteData(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);	        //??????
			SX1276WriteData(REG_LR_HOPPERIOD, PACKET_MIAX_Value );      //0x24???,???????????
			SX1276WriteData( REG_LR_DIOMAPPING1, 0X02 );                  //PayloadCrcError???DIO3
			SX1276WriteData( REG_LR_DIOMAPPING2, 0x00 );	
			SX1276LoRaSetOpMode(Recevived_mode);
		}
		else
		{						   
		  //û��⵽
			//SX1276LoRaSetOpMode(stdby_mode);
			SX1276WriteData(REG_LR_IRQFLAGSMASK,IRQN_SEELP_Value);	//??????
			SX1276WriteData( REG_LR_DIOMAPPING1, 0X00);
			SX1276WriteData( REG_LR_DIOMAPPING2, 0X00);	
			SX1276LoRaSetOpMode(sleep_mode);
		}
	 }
	  else
	 {
		SX1276LoRaSetOpMode(stdby_mode);
		SX1276WriteData(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);	        //??????
		SX1276WriteData(REG_LR_HOPPERIOD,   PACKET_MIAX_Value );      //0x24???,???????????
		SX1276WriteData( REG_LR_DIOMAPPING1, 0X02 );                  //PayloadCrcError???DIO3
		SX1276WriteData( REG_LR_DIOMAPPING2, 0x00 );	
		SX1276LoRaSetOpMode(Recevived_mode);
		
	 }
	
      SX1276WriteData(REG_LR_IRQFLAGS,0xff);//
			//TIM2->CR1&=~TIM_CR1_CEN;//�رն�ʱ��
}



/*
 * ��������TIM2_NVIC_Configuration
 * ����  ��TIM2�ж����ȼ�����
 * ����  ����
 * ���  ����	
 */
void TIM2_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}



/*
���´���ΪSX1278��λ��ʼ��
*/

void reset_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //��PB��ʱ��
 // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//��PE��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//PB5,PE5��������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	//�˿��ٶ�
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//�˿�ģʽ����Ϊ�������ģʽ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//��ʼ����Ӧ�Ķ˿�
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/*
 * ��������TX_RX_LED(void)
 * ����  ��ָʾ�����շ�
 * ����  ����
 * ���  ����	
 */

void TX_RX_LED(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //��PC��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	//PB5,PE5��������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_9;
	//�˿��ٶ�
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//�˿�ģʽ����Ϊ�������ģʽ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//��ʼ����Ӧ�Ķ˿�
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	LED_TX_ON;
	LED_RX_ON;
	LED4_OFF;//�رղఴ����
	Key();
	
}
/*
 * ��������void Key(void)
 * ����  ���ఴ��
 * ����  ����
 * ���  ����	
 */
void Key(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //��PC��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//PB5,PE5��������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	//�˿��ٶ�
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//�˿�ģʽ����Ϊ�������ģʽ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//��ʼ����Ӧ�Ķ˿�
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*
�������ƣ�RecvData(unsigned char *buf,SX1278_RLEN);
����    :�������ݽ���
����    ����
����ֵ  ����
autor   : niub 
*/
void fqcRecvData(unsigned char *buf,unsigned short SX1278_RLEN)// 
{



}
