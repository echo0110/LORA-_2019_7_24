/*-------------------------------------------------------------------------------
文件名称：main.c
文件描述：SPI FLASH 基本测试
硬件平台：尼莫M3S开发板
编写整理：shifang
固件库  ：V3.5
技术论坛：www.doflye.net
备    注：通过简单修改可以移植到其他开发板，部分资料来源于网络。
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
#include "led.h"   //timer在这个函数里面
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
#define  Write_channel_temp_ADDR   0x08000000+100*1024//设定固定频段 断电保存
//要写入到SPI Flash的字符串数组
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
void config_mode(void);//配置模式
void test_mode();//测试模式
void touchuan_mode(void);//透传模式
int main(void)  //复位-->初始化-->发射-->接收
{		
	u8 key;
	u16 i=0;	
	u32 FLASH_SIZE;	
//	LED_Init();//LED初始化
  SX1276_IO_Init();
	reset_Init();//复位初始化引脚
 	TX_RX_LED();//LED指示收发初始化
  uart_init(9600);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断优先级分组2  2位抢占  2位响应
  /*TIM2定时配置*/
	//TIM2_NVIC_Configuration();
	SysTick_Init();
	Timer2_Init(10-1, 7200-1);//以10KHZ的频率计数 计数到5000为500ms  计数到10为1ms
	//Timer3_Init(12000-1,7200-1);
	sx1278_init();	
	STMFLASH_Write(Write_channel_temp_ADDR, (u16*)&rate_buff[15],1);
	channel_temp=read_channel_temp();
  printf("\r\n  system initial done..... \r\n");
  	while(1)
  {	
		//IWDG_Feed(); //看门dog			
		if(Usart1_Over)
		{
			Usart1_Over=0;
     if(config==1)	
		  {			
			 {		
        uart_init(9600);//配置模式9600bps				 
			  config_mode();
				LED_RX_ON;//关闭LED-TX
				Usart1_Rec_Cnt=0;							
			 }
		  }
			else            //透传模式的话  读flash设的bps
			{			
			 touchuan_mode();
			}				
		}
    else if(SX1276ReadData(REG_LR_IRQFLAGS))  //读0x12寄存器---收到下面的ＬＯＲＡ回的数据
	  { 
 			 SetTxPacket(); ;//以10KHZ的频率计数 计数到5000为500ms  计数到5为0.5ms  //进去后要关下定时器	   
	  }	
	}			
}


#if   defined(SX1278_RX)   //

void  test_mode(void)
{
    set_air_speed2();	
}

#endif


 
 


