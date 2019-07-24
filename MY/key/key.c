/*-------------------------------------------------------------------------------
�ļ����ƣ�key.c
�ļ����������ð�����ʼ������       
��    ע����
---------------------------------------------------------------------------------*/
#include "key.h"
#include "config.h"
#include "delay.h"
#include  "SX1276.h"
#include  "led.h"
unsigned char Trg;//����
unsigned char Cont;//��������
#define KEY_MODE 0x01    //ģʽ����
#define KEY_PLUS 0x02     // ��
unsigned char config=0;


/*-------------------------------------------------------------------------------
�������ƣ�KEY_Init
�����������������ų�ʼ�����򿪶˿�ʱ�ӣ����ö˿����ţ��˿ڹ���Ƶ�ʣ��˿�����ģʽ 
�����������
���ز�������
��    ע����
---------------------------------------------------------------------------------*/
void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	//��PB��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//��PA��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//��PE��ʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	//PE2,PE3,PE4��������	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	//�˿��ٶ�
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	//�˿�ģʽ����Ϊ��������ģʽ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	//��ʼ����Ӧ�Ķ˿�
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	//PA0��������	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	//�˿��ٶ�
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	//�˿�ģʽ����Ϊ��������ģʽ
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	//��ʼ����Ӧ�Ķ˿�
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

u8 KEY_Scan(u8 mode)
{	 
//	static u8 key_up=1;//�������ɿ���־
//	if(mode)key_up=1;  //֧������		  
//	if(key_up&&(S1==0||S2==0||S3==0||S4==1))
//	{
//		Delay_ms(10);//ȥ���� 
//		key_up=0;
//		if(S1==0)return 1;
//		else if(S2==0)return 2;
//		else if(S3==0)return 3;
//		else if(S4==1)return 4;
//	}else if(S1==1&&S2==1&&S3==1&&S4==0)key_up=1; 	    
// 	return 0;// �ް�������
}

/*-------------------------------------------------------------------------------
�������ƣ�KeyProc(void)
��������������ɨ��
�����������
���ز�������
��    ע����
*/
void KeyProc(void)
{	 
	static int Flag,cnt_plus=0;	
  switch(config)
	{	   	
		case 0://͸��ģʽ		
			   if(Cont) //����
				 { 
					 cnt_plus++;	
					 if((cnt_plus>=2000)&&(cnt_plus<5000))
					 { LED4_ON;
					  //printf("%d\n",cnt_plus);
					 }
					 if(cnt_plus>=5000)
					 {
					   printf("%d\n",cnt_plus);						
						 j=1;						 
					 }
					 Flag=1;				 
				 }
				 if((!Cont)&&(Flag==1))
				 {
					  if(cnt_plus>2000&&cnt_plus<5000)
						{
							printf("\r\n realse.......\r\n");
							config=1;
							cnt_plus=0;
						}
            else if(cnt_plus>=5000)
						{NVIC_SystemReset();}							
				 }
		  break;
	  case 1://����ģʽ
			  if(Cont)
				{
					cnt_plus++;
					if((cnt_plus>=2000)&&(cnt_plus<5000))
					{
						LED4_OFF;
					 // printf("\r\n 2S--OFF.......\r\n");
						Flag=1;
					}
				}
				if((!Cont)&&(Flag==1))
				{
					if(cnt_plus>2000&&cnt_plus<5000)
					{
					//	printf("\r\n realse--2.......\r\n");						
						config=0;
						cnt_plus=0;
					}           					
				}
			break;
 	}
}
/*-------------------------------------------------------------------------------
�������ƣ�KeyRead(void)
����������������״̬
�����������
���ز�������
��    ע����
*/
void KeyRead(void)  
{
 unsigned char ReadData=(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4))^0x01;
 Trg=ReadData&(ReadData^Cont);
 Cont=ReadData;
}


/*----------------------�·��� ������̳��www.doflye.net--------------------------*/
