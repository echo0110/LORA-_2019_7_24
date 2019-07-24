/*-------------------------------------------------------------------------------
�ļ����ƣ�main.c
�ļ�������SPI FLASH ��������
Ӳ��ƽ̨����ĪM3S������
��д����shifang
�̼���  ��V3.5
������̳��www.doflye.net
��    ע��ͨ�����޸Ŀ�����ֲ�����������壬����������Դ�����硣
---------------------------------------------------------------------------------*/
#include <stdio.h>
#include "string.h"
#include "stm32f10x.h"
#include "stmflash.h"
#include "103.h"
#include "usart.h"
#include "SX1276.h"
#include "iwdg.h"
//#include  "SysTick_Init"
#include "led.h"   //timer�������������
#include "Delay.h"
#include "key.h"
#include "timer.h"
#include "beep.h"
#include "usart.h"
#include "adc.h"
#include "24cxx.h" 
#include "flash.h" 
#include "bsp_usart.h"
#include  "spi.h"
#include "config.h"
#define  TEST  0
#define  Write_channel_temp_ADDR   0x08000000+100*1024//�趨�̶�Ƶ�� �ϵ籣��
//Ҫд�뵽SPI Flash���ַ�������
const u8 TEXT_Buffer[]={{7},{0},{19}};
#if  (TEST)
#define   SIZE             sizeof(TEXT_Buffer[3])
#else

#endif


#define  SX1278_RX  1
#define  SX1278_TX  1
#define  DEBUG      0
  
u16  temp_buf[300];



//u8 datatemp[SIZE];
void config_mode(void);//����ģʽ
void test_mode();//����ģʽ
void touchuan_mode(void);//͸��ģʽ
int main(void)  //��λ-->��ʼ��-->����-->����
{		
	u8 key;
	u16 i=0;	
	u32 FLASH_SIZE;	
//	LED_Init();//LED��ʼ��
  SX1276_IO_Init();
	reset_Init();//��λ��ʼ������
 	TX_RX_LED();//LEDָʾ�շ���ʼ��
  uart_init(9600);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����ж����ȼ�����2  2λ��ռ  2λ��Ӧ
  /*TIM2��ʱ����*/
	//TIM2_NVIC_Configuration();
	SysTick_Init();
	Timer2_Init(10-1, 7200-1);//��10KHZ��Ƶ�ʼ��� ������5000Ϊ500ms  ������10Ϊ1ms
	//Timer3_Init(12000-1,7200-1);
	sx1278_init();	
	STMFLASH_Write(Write_channel_temp_ADDR, (u16*)&rate_buff[15],1);
	channel_temp=read_channel_temp();
  printf("\r\n  system initial done..... \r\n");
  	while(1)
  {	
		//IWDG_Feed(); //����dog			
		if(Usart1_Over)
		{
			Usart1_Over=0;
     if(config==1)	
		  {			
			 {		
        uart_init(9600);//����ģʽ9600bps				 
			  config_mode();
				LED_RX_ON;//�ر�LED-TX
				Usart1_Rec_Cnt=0;							
			 }
		  }
			else            //͸��ģʽ�Ļ�  ��flash���bps
			{			
			 touchuan_mode();
			}				
		}
    else if(SX1276ReadData(REG_LR_IRQFLAGS))  //��0x12�Ĵ���---�յ�����ģ̣ϣң��ص�����
	  { 
 			 SetTxPacket(); ;//��10KHZ��Ƶ�ʼ��� ������5000Ϊ500ms  ������5Ϊ0.5ms  //��ȥ��Ҫ���¶�ʱ��	   
	  }	
	}			
}


#if   defined(SX1278_RX)   //

void  test_mode(void)
{
    set_air_speed2();	
}

#endif


 
 


