/*-------------------------------------------------------------------------------
文件名称：key.c
文件描述：配置按键初始化参数       
备    注：无
---------------------------------------------------------------------------------*/
#include "key.h"
#include "config.h"
#include "delay.h"
#include  "SX1276.h"
#include  "led.h"
unsigned char Trg;//触发
unsigned char Cont;//连续按下
#define KEY_MODE 0x01    //模式按键
#define KEY_PLUS 0x02     // 加
unsigned char config=0;


/*-------------------------------------------------------------------------------
程序名称：KEY_Init
程序描述：按键引脚初始化，打开端口时钟，配置端口引脚，端口工作频率，端口输入模式 
输入参数：无
返回参数：无
备    注：无
---------------------------------------------------------------------------------*/
void KEY_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	//打开PB口时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	//打开PA口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	//打开PE口时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	//PE2,PE3,PE4引脚设置	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	//端口速度
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	//端口模式，此为输入上拉模式
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	//初始化对应的端口
  GPIO_Init(GPIOE, &GPIO_InitStructure);
	//PA0引脚设置	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	//端口速度
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	//端口模式，此为输入下拉模式
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	//初始化对应的端口
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

u8 KEY_Scan(u8 mode)
{	 
//	static u8 key_up=1;//按键按松开标志
//	if(mode)key_up=1;  //支持连按		  
//	if(key_up&&(S1==0||S2==0||S3==0||S4==1))
//	{
//		Delay_ms(10);//去抖动 
//		key_up=0;
//		if(S1==0)return 1;
//		else if(S2==0)return 2;
//		else if(S3==0)return 3;
//		else if(S4==1)return 4;
//	}else if(S1==1&&S2==1&&S3==1&&S4==0)key_up=1; 	    
// 	return 0;// 无按键按下
}

/*-------------------------------------------------------------------------------
程序名称：KeyProc(void)
程序描述：按键扫描
输入参数：无
返回参数：无
备    注：无
*/
void KeyProc(void)
{	 
	static int Flag,cnt_plus=0;	
  switch(config)
	{	   	
		case 0://透传模式		
			   if(Cont) //长按
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
	  case 1://配置模式
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
程序名称：KeyRead(void)
程序描述：读按键状态
输入参数：无
返回参数：无
备    注：无
*/
void KeyRead(void)  
{
 unsigned char ReadData=(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4))^0x01;
 Trg=ReadData&(ReadData^Cont);
 Cont=ReadData;
}


/*----------------------德飞莱 技术论坛：www.doflye.net--------------------------*/
