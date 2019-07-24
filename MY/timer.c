#include "stm32f10x.h"
#include "SX1276.h"
#include "timer.h"
#include  "stdio.h"

unsigned char   RF_EX0_STATUS; //RF中断标志寄存器
unsigned char   CRC_Value;
unsigned char t;
unsigned char recv[10];
unsigned char   SX1278_RLEN;



void Timer2_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能	 
//	TIM_TimeBaseStructure.TIM_Period = 5000; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
//	TIM_TimeBaseStructure.TIM_Prescaler =(7200-1); //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

//	TIM_ITConfig(  //使能或者失能指定的TIM中断
//		TIM2, //TIM2
//		TIM_IT_Update  |  //TIM 中断源
//		TIM_IT_Trigger,   //TIM 触发中断源 
//		ENABLE  //使能
//		);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	TIM_ClearFlag(TIM2,TIM_FLAG_Update);

//	TIM_Cmd(TIM2, ENABLE);  //使能TIMx外设							 
}


void Timer2_enable( void )
{
	TIM_ClearFlag(TIM2, TIM_FLAG_Update); //清除溢出中断标志
	TIM_SetCounter(TIM2,0x00);			//清零计数器值
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_Trigger,ENABLE);
	TIM_Cmd(TIM2,ENABLE);
}

void Timer2_disable (void)
{
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE); //时钟失能
//	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_Trigger,DISABLE );
	TIM_Cmd(TIM2, DISABLE);  //失能TIMx外设
}



void TIM2_IRQHandler(void)   //TIM2中断
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
//		LED0=!LED0;									   //接收完成闪灯
		RF_EX0_STATUS=SX1276ReadData(REG_LR_IRQFLAGS); //读取0x12中断标志寄存器
		if(RF_EX0_STATUS&&0x40==0x40)
		{
		 CRC_Value=SX1276ReadData(REG_LR_MODEMCONFIG2 );//计算CRC
			if(CRC_Value&&0x04==0x04)
			{
			 SX1276WriteData(REG_LR_FIFOADDRPTR,0x00);            //??SPI???FIFO?????????
			 SX1278_RLEN = SX1276ReadData(REG_LR_NBRXBYTES); //读取最后一包的字节数
			 switch_cs(enOpen);
			 RF_SPI_MasterIO(0x00);//突发访问 读
				for(t=0;t<SX1278_RLEN;t++)
				{
				  recv[t]=RF_SPI_READ_BYTE();	
				}
				 switch_cs(enClose);
			}
			if((recv[0]==0x5a)&(recv[2]==0xfa))  //前后数据都接收到了才算正确
		{
        // fqcRecvData(recv,SX1278_RLEN);  //处理接收到的数据
		}
		
		}
		Timer2_disable();
		
	}
}


















