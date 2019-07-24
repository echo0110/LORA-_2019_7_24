/*-------------------------------------------------------------------------------
文件名称：led.c
文件描述：根据硬件连接配置LED端口，打开对应的寄存器        
备    注：无
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





 u8  temp_buf2[255];//存无线接收数据
unsigned char  RF_EX0_STATUS; //RF
unsigned char  x4,x5; //RF
unsigned char   CRC_Value;
unsigned char recv[255];
unsigned char   SX1278_RLEN;
unsigned char t;
volatile unsigned char timer;
int j=0;
/*-------------------------------------------------------------------------------
程序名称：LED_Init
程序描述：初始化LED相关端口，打开端口时钟，配置端口输出  
输入参数：无
返回参数：无
备    注：无
---------------------------------------------------------------------------------*/
void LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //打开PA口时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//PB5,PE5引脚设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	//端口速度
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//端口模式，此为输出推挽模式
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//初始化对应的端口
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
}

void Timer2_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能	 
	TIM_TimeBaseStructure.TIM_Period = arr;//重装值 //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc;//分频值 //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //把上述值写入相应的寄存器
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);  //使能或者失能指定的TIM中断
		

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //把上述值写入相应的寄存器
 	TIM_Cmd(TIM2, ENABLE);  //使能TIMx外设							 
}

//通用定时器中断初始化	通用定时器 2、3、4
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器2,3!
void Timer3_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能	  
//	TIM_TimeBaseStructure.TIM_Period = 5000; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
//	TIM_TimeBaseStructure.TIM_Prescaler =(7200-1); //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
//	TIM_ITConfig(  //使能或者失能指定的TIM中断
//		TIM3, //TIM3
//		TIM_IT_Update  |  //TIM 中断源
//		TIM_IT_Trigger,   //TIM 触发中断源 
//		ENABLE  //使能
//		);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_ClearFlag(TIM3, TIM_FLAG_Update); //清除溢出中断标志

//	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
}

void Timer3_enable( void )
{
	TIM_ClearFlag(TIM3, TIM_FLAG_Update); //清除溢出中断标志
	TIM_SetCounter(TIM3,0x00);			//清零计数器值
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
}

void Timer3_disable (void)
{
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE); //时钟失能
//	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
	TIM_ITConfig(TIM3, TIM_IT_Update | TIM_IT_Trigger,DISABLE );
	TIM_Cmd(TIM3, DISABLE);  //失能TIMx外设
}


/*-------------------------------------------------------------------------------
程序名称：TIM2_IRQHandler(void)
程序描述：TIM2中断
输入参数：无
返回参数：无
备    注：无
*/
void TIM2_IRQHandler(void)   //TIM2中断
{ 	
  static int Count=0; 	
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的中断发生与否
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清楚TIMX的中断处理位	 
			
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
		
		//timer++; // 1ms中断一次,
		
	//	TIM2->CR1&=~TIM_CR1_CEN;//关闭定时器
	}
}

/*-------------------------------------------------------------------------------
程序名称：TIM3_IRQHandler(void)
程序描述：TIM3中断
输入参数：无
返回参数：无
备    注：无
*/
void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
		//Timer3_disable();
	
		Timer3_enable();
		 
	}
}
/*-------------------------------------------------------------------------------
程序名称：void led_toggle();
程序描述：LED电平翻转
输入参数：无
返回参数：无
备    注：无
*/
void led_toggle(void)
{
	LED4_REV;		
}




 void SetTxPacket(void)
{  
		  
		    RF_EX0_STATUS=SX1276ReadData(REG_LR_IRQFLAGS);		  
		   if((RF_EX0_STATUS&0x40)==0x40)  //接收完成   
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
				 LED_TX_OFF;//点亮收无线指示LED-RX
		    UART_Send(recv,SX1278_RLEN);//收无线  通过串口发出去
				SX1278_RLEN=0;//清零
				LED_TX_ON;//关闭LED-RX
			}
			SX1276LoRaSetOpMode(stdby_mode);
	    SX1276WriteData(REG_LR_IRQFLAGSMASK, IRQN_RXD_Value);    
		  SX1276WriteData(REG_LR_HOPPERIOD, PACKET_MIAX_Value);//0x24  
		  SX1276WriteData( REG_LR_DIOMAPPING1, 0X00 ); // ? ?  ? ?I  
		  SX1276WriteData( REG_LR_DIOMAPPING2, 0x00 );	
		  SX1276LoRaSetOpMode(Recevived_mode);
		}
			
    }
		 else if((RF_EX0_STATUS&0x08)==0x08)  //发射完成
	  {//发射完成后,置为接收模式
		 // SX1276LoRaSetOpMode(stdby_mode );
		 SX1276WriteData(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);	        //??????
		 SX1276WriteData(REG_LR_HOPPERIOD,   PACKET_MIAX_Value );      //0x24???,???????????
		 SX1276WriteData( REG_LR_DIOMAPPING1, 0X00 );
		 SX1276WriteData( REG_LR_DIOMAPPING2, 0x00 );	
		 SX1276LoRaSetOpMode(Recevived_mode);
		 SX1276WriteData( REG_LR_FIFOTXBASEADDR, 0);           //?Tx FIFO??
	   SX1276WriteData( REG_LR_FIFOADDRPTR, 0 ); 
		
	  }
		else if((RF_EX0_STATUS&0x04)==0x04)  //cad完成  (这个应该是另一种接收方式)
	 {  
		if((RF_EX0_STATUS&0x01)==0x01)
		{	 
		//表示CAD检测到扩频信号 模块进入接收模式
			SX1276LoRaSetOpMode( stdby_mode );
			SX1276WriteData(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);	        //??????
			SX1276WriteData(REG_LR_HOPPERIOD, PACKET_MIAX_Value );      //0x24???,???????????
			SX1276WriteData( REG_LR_DIOMAPPING1, 0X02 );                  //PayloadCrcError???DIO3
			SX1276WriteData( REG_LR_DIOMAPPING2, 0x00 );	
			SX1276LoRaSetOpMode(Recevived_mode);
		}
		else
		{						   
		  //没检测到
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
			//TIM2->CR1&=~TIM_CR1_CEN;//关闭定时器
}



/*
 * 函数名：TIM2_NVIC_Configuration
 * 描述  ：TIM2中断优先级配置
 * 输入  ：无
 * 输出  ：无	
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
以下代码为SX1278复位初始化
*/

void reset_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //打开PB口时钟
 // RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//打开PE口时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//PB5,PE5引脚设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	//端口速度
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//端口模式，此为输出推挽模式
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//初始化对应的端口
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  //GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/*
 * 函数名：TX_RX_LED(void)
 * 描述  ：指示数据收发
 * 输入  ：无
 * 输出  ：无	
 */

void TX_RX_LED(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //打开PC口时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	//PB5,PE5引脚设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_9;
	//端口速度
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//端口模式，此为输出推挽模式
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//初始化对应的端口
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	LED_TX_ON;
	LED_RX_ON;
	LED4_OFF;//关闭侧按键灯
	Key();
	
}
/*
 * 函数名：void Key(void)
 * 描述  ：侧按键
 * 输入  ：无
 * 输出  ：无	
 */
void Key(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  //打开PC口时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//PB5,PE5引脚设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	//端口速度
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//端口模式，此为输出推挽模式
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//初始化对应的端口
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}


/*
函数名称：RecvData(unsigned char *buf,SX1278_RLEN);
功能    :串口数据接收
参数    ：无
返回值  ：无
autor   : niub 
*/
void fqcRecvData(unsigned char *buf,unsigned short SX1278_RLEN)// 
{



}
