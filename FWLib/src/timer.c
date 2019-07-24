#include "stm32f10x.h"
#include "SX1276.h"
#include "timer.h"
#include  "stdio.h"

unsigned char   RF_EX0_STATUS; //RF�жϱ�־�Ĵ���
unsigned char   CRC_Value;
unsigned char t;
unsigned char recv[10];
unsigned char   SX1278_RLEN;



void Timer2_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʱ��ʹ��	 
//	TIM_TimeBaseStructure.TIM_Period = 5000; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
//	TIM_TimeBaseStructure.TIM_Prescaler =(7200-1); //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������5000Ϊ500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

//	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
//		TIM2, //TIM2
//		TIM_IT_Update  |  //TIM �ж�Դ
//		TIM_IT_Trigger,   //TIM �����ж�Դ 
//		ENABLE  //ʹ��
//		);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

	TIM_ClearFlag(TIM2,TIM_FLAG_Update);

//	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIMx����							 
}


void Timer2_enable( void )
{
	TIM_ClearFlag(TIM2, TIM_FLAG_Update); //�������жϱ�־
	TIM_SetCounter(TIM2,0x00);			//���������ֵ
	TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_Trigger,ENABLE);
	TIM_Cmd(TIM2,ENABLE);
}

void Timer2_disable (void)
{
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE); //ʱ��ʧ��
//	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_Trigger,DISABLE );
	TIM_Cmd(TIM2, DISABLE);  //ʧ��TIMx����
}



void TIM2_IRQHandler(void)   //TIM2�ж�
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ 
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ 
//		LED0=!LED0;									   //�����������
		RF_EX0_STATUS=SX1276ReadData(REG_LR_IRQFLAGS); //��ȡ0x12�жϱ�־�Ĵ���
		if(RF_EX0_STATUS&&0x40==0x40)
		{
		 CRC_Value=SX1276ReadData(REG_LR_MODEMCONFIG2 );//����CRC
			if(CRC_Value&&0x04==0x04)
			{
			 SX1276WriteData(REG_LR_FIFOADDRPTR,0x00);            //??SPI???FIFO?????????
			 SX1278_RLEN = SX1276ReadData(REG_LR_NBRXBYTES); //��ȡ���һ�����ֽ���
			 switch_cs(enOpen);
			 RF_SPI_MasterIO(0x00);//ͻ������ ��
				for(t=0;t<SX1278_RLEN;t++)
				{
				  recv[t]=RF_SPI_READ_BYTE();	
				}
				 switch_cs(enClose);
			}
			if((recv[0]==0x5a)&(recv[2]==0xfa))  //ǰ�����ݶ����յ��˲�����ȷ
		{
        // fqcRecvData(recv,SX1278_RLEN);  //������յ�������
		}
		
		}
		Timer2_disable();
		
	}
}


















