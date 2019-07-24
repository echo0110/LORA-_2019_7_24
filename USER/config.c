/******************** (C) COPYRIGHT 2012 WildFire Team ***************************
 * �ļ���  ��key.c
 * ����    ������Ӧ�ú�����         
 * ʵ��ƽ̨��Ұ��STM32������
 * Ӳ�����ӣ�-------------------------
 *          | PE5 - key1              |
 *          | PE6 - key2(��ʵ��û�õ�)|
 *          |                         |
 *           -------------------------
 * ��汾  ��ST3.5.0
 *
 * ����    ��wildfire team 
 * ��̳    ��http://www.amobbs.com/forum-1008-1.html
 * �Ա�    ��http://firestm32.taobao.com
**********************************************************************************/
#include "config.h"
#include "usart.h"
#include "bsp_usart.h"
#include  "SX1276.h"
#include "stmflash.h"
#include "103.h"
#include "led.h"
#include "delay.h"
#define  Write_START_ADDR   0x08000000+50*1024//0x08070000//0x8000000+100*1024
#define  Write_BPS_ADDR   0x08000000+60*1024//0x08070000//0x8000000+100*1024
#define  Write_air_speed_ADDR   0x08000000+80*1024//�������ʵ�ַ
#define  Write_chnnel_ADDR   0x08000000+80*1024//�ŵ���ַ

#define   SIZE             sizeof(rate_buff[0])	
#define   SIZE2            sizeof(baud[0])	
#define   SIZE3            sizeof(bps_buff[0])                 //У��SIZE
u8 buf[10]={0};	//�������
u8 buff_1200[10]={0xC0,0x12,0x34,0x04,0x17,0x44};//1200 bps
u8 buff_2400[10]={0xC0,0x12,0x34,0x0C,0x17,0x44};//2400 bps
u8 buff_4800[10]={0xC0,0x12,0x34,0x14,0x17,0x44};//4800 bps
u8 buff_9600[10]={0xC0,0x12,0x34,0x1C,0x17,0x44};//9600 bps
u8 buff_19200[10]={0xC0,0x12,0x34,0x24,0x17,0x44};//19200 bps
u8 C5_buff[10]={0xC5,0x0C,0x1C};
u16 temp_buff[SIZE];//��ʱ����� ��flash���Ĳ���
u16  baud[10]={1200,2400,4800,9600,19200};
u16 crcData;//
static int k;
u8 flash_test[30];//just  for read flash test
u8 CR;
u8 channel_temp;//�ϵ��Ƶ��channel_tempȻ��channel_temp->DMA_Rece_Buf[4]
 


struct rate  rate_buff[30]=
{
 {7,1,9},{7,4,9},{7,3,9},{9,3,9},{7,5,9},{10,4,9},{12,1,9},{7,1,9},{7,3,1},{7,1,9},
 {7,2,7},{9,1,9},{10,1,9},{11,1,9},{12,1,9},{0x17,0x17,0x17}
};
struct rate  datatemp[10];//��ȡ����ʱ�� ��ʱ�洢
  struct Freq  Fre[33]=    //Ƶ��  /*410-411-412-413-414*/
{                                 /*415-416-417-418-419*/ 
                                  /*420-421-422-423-424*/
                                  /*425-426-427-428-429*/ 
                                  /*430-431-432-433-434*/	
	                                /*435-436-437-438-439*/
	                                /*440-441*/
 {0x66,0x80,0x11},{0x66,0xC0,0x11},{0x67,0x00,0x11},{0x67,0x40,0x11},{0x67,0x80,0x11},
 {0x67,0xC0,0x11},{0x68,0x00,0x11},{0x68,0x40,0x11},{0x68,0x80,0x11},{0x68,0xC0,0x11},
 {0x69,0x00,0x11},{0x69,0x40,0x11},{0x69,0x80,0x11},{0x69,0xC0,0x11},{0x6a,0x00,0x11},
 {0x6a,0x40,0x11},{0x6a,0x80,0x11},{0x6a,0xc0,0x11},{0x6B,0x00,0x11},{0x6B,0x40,0x11},
 {0x6B,0x80,0x12},{0x6B,0xC0,0x12},{0x6c,0x00,0x12},{0x6c,0x80,0x00},{0x6c,0x80,0x12},
 {0x6c,0xC0,0x12},{0x6d,0x00,0x12},{0x6d,0x40,0x12},{0x6d,0x80,0x12},{0x6d,0xc0,0x12},
 {0x6e,0x00,0x12},{0x6e,0x40,0x12}
};
struct bps   bps_buff[10]={0};//bps�ṹ������
/*
 * ��������config_mode(void)
 * ����  ������ģʽ
 * ����  ����
 * ���  ����	
 */
void config_mode(void)
{	
   Read_flash();//�ȶ�flashֵ	
	 LED_RX_OFF;//����LED-TX
	//if(DMA_Rece_Buf[1]==0xC0)//
		switch(DMA_Rece_Buf[0])
		{
		  case 0xC0: //����ģ�����--�ϵ�ɱ���
				        channel_temp=DMA_Rece_Buf[4];//�ϵ��  ��Ҫ���ĵ��ŵ�  ���ǵĻ� ��Ĭ��23
				        switch(DMA_Rece_Buf[3])
								{
									case 0x04:	
                    bpsCRC(baud,0);										
										STMFLASH_Write(Write_BPS_ADDR, (u16*)&bps_buff[0],4);//1200bps
                    printf("OK\r\n"); 
								  	break;
									case 0x0C:
										bpsCRC(baud,1);
										STMFLASH_Write(Write_BPS_ADDR, (u16*)&bps_buff[1],4);//2400bps
									  printf("OK\r\n"); 
										break;
									case 0x14:
										bpsCRC(baud,2);
										STMFLASH_Write(Write_BPS_ADDR, (u16*)&bps_buff[2],4);//4800bps
									  printf("OK\r\n"); 
										break;										
									case 0x18:
							    	bpsCRC(baud,3);
										STMFLASH_Write(Write_BPS_ADDR, (u16*)&bps_buff[3],4);//9600bps
										//SX1276LoRaSetRFchannel(DMA_Rece_Buf[4]); 
									  set_air_speed(6);//0.3K 	
                    STMFLASH_Write(Write_air_speed_ADDR, (u16*)&rate_buff[6],SIZE);//��������дflash										
										printf("OK\r\n"); 					
									  break;
									case 0x19: //9600-1.2K
									    bpsCRC(baud,3);
											STMFLASH_Write(Write_BPS_ADDR, (u16*)&bps_buff[3],4);//9600bps
									   // SX1276LoRaSetRFchannel(DMA_Rece_Buf[4]);
    									set_air_speed(5);//1.2K 
                      STMFLASH_Write(Write_air_speed_ADDR, (u16*)&rate_buff[5],SIZE);//��������дflash								
	                    printf("OK\r\n"); 																				 
									break;
									case 0x1A:
									  bpsCRC(baud,3);
								  	STMFLASH_Write(Write_BPS_ADDR, (u16*)&bps_buff[3],4);//9600bps
					          set_air_speed(10);// 2.4K
									// STMFLASH_Write(Write_START_ADDR, (u16*)&rate_buff[1],3);
									 STMFLASH_Write(Write_air_speed_ADDR, (u16*)&rate_buff[10],SIZE);//��������дflash
									  printf("OK\r\n");           //�ָ���������
									     break;
									case 0x1B:
										bpsCRC(baud,3);
								  	STMFLASH_Write(Write_BPS_ADDR, (u16*)&bps_buff[3],4);//9600bps
									// 	SX1276LoRaSetRFchannel(DMA_Rece_Buf[4]);//���ŵ�
								   	set_air_speed(3);//4.8K--	
									  STMFLASH_Write(Write_air_speed_ADDR, (u16*)&rate_buff[3],SIZE);//��������дflash
                   // STMFLASH_Write(Write_START_ADDR, (u16*)&rate_buff[3],3);									
									  printf("OK\r\n");           //�ָ���������								
									      break;
									case 0x1C: //9600-9.6K
										bpsCRC(baud,3);
								  	STMFLASH_Write(Write_BPS_ADDR, (u16*)&bps_buff[3],4);//9600bps									  								
									  SX1276LoRaSetRFchannel(DMA_Rece_Buf[4]);		
  									set_air_speed(1);//9.6K 
									  STMFLASH_Write(Write_air_speed_ADDR, (u16*)&rate_buff[1],SIZE);//��������дflash
									  SX1276LoRaSetRFchannel(DMA_Rece_Buf[4]);//���ŵ�								  
									 // STMFLASH_Write(Write_chnnel_ADDR, (u16*)&DMA_Rece_Buf[4],1);//��õ��ŵ�д��flash
  									printf("OK\r\n");		                  							
									     break;
								  case 0x1D:
										bpsCRC(baud,3);
								  	STMFLASH_Write(Write_BPS_ADDR, (u16*)&bps_buff[3],4);//9600bps
									  //SX1276LoRaSetRFchannel(DMA_Rece_Buf[4]);//���ŵ�
     								set_air_speed(9);//19.2K 
                    STMFLASH_Write(Write_air_speed_ADDR, (u16*)&rate_buff[9],SIZE);//��������дflash								
									  printf("OK\r\n");	
								       break;		
 /***********************************9600bps-19200bps�����ķָ���****************************************************************/						
                  case 0x20: //19200-292bit/s
									set_air_speed(5);		
									STMFLASH_Write(Write_START_ADDR, (u16*)&rate_buff[0],SIZE);//12,1,7--292bit/s																															
									break;
									case 0x21: //19200-1.2K
								  set_air_speed(1);	//7,4,9--1.3K/s	
									STMFLASH_Write(Write_START_ADDR, (u16*)&rate_buff[0],SIZE);//					
									break;
									case 0x22: //19200-2.4K	
										bpsCRC(baud,4);
									  STMFLASH_Write(Write_BPS_ADDR, (u16*)&bps_buff[4],4);//19200bps						
								    printf("OK\r\n");
									break;
									case 0x23: //19200-4.8K	
									  bpsCRC(baud,4);
								  	STMFLASH_Write(Write_BPS_ADDR, (u16*)&bps_buff[4],4);//19200bps
								   	set_air_speed(3);//4.8K--	
									  STMFLASH_Write(Write_air_speed_ADDR, (u16*)&rate_buff[3],SIZE);//��������дflash
                   // STMFLASH_Write(Write_START_ADDR, (u16*)&rate_buff[3],3);									
									  printf("OK\r\n");           //�ָ���������	
									break;
										case 0x24: //19200-9.6K	
										bpsCRC(baud,4);
								  	STMFLASH_Write(Write_BPS_ADDR, (u16*)&bps_buff[4],4);//19200bps									  								
  									set_air_speed(1);//9.6K 
									  STMFLASH_Write(Write_air_speed_ADDR, (u16*)&rate_buff[1],SIZE);//��������дflash
									  printf("OK\r\n");	
										break;
										case 0x25: //19200-19.2K	
										bpsCRC(baud,4);
								  	STMFLASH_Write(Write_BPS_ADDR, (u16*)&bps_buff[4],4);//19200bps
     								set_air_speed(0);//19.2K 
                    STMFLASH_Write(Write_air_speed_ADDR, (u16*)&rate_buff[0],SIZE);//��������дflash								
									  printf("OK\r\n");	
									break;
									default: break; 										
								}
					 break;
			case 0xC1://��ȡģ�����
				       switch(DMA_Rece_Buf[1])
							  {
							    case 0xC1:
										       switch(DMA_Rece_Buf[2])
													 {
														 case 0xC1://��ȡ���ò���
															 Read_flash();//�ȶ�flashֵ	
														 // STMFLASH_Read(Read_air_spped_ADDR,(u16*)flash_test,3);
														 // printf("CR=%d\n",flash_test[1]);
															  LED_TX_OFF;//
														     switch(crc_flash)
																 {
																	 case 1200:
																		        buff_1200[3]=read_air_speed(0);//0x04;//DMA_Rece_Buf[3];
																		        buff_1200[4]=channel_temp;//DMA_Rece_Buf[4];
																	          UART_Send((u8*)buff_1200,6);
																		 break;
																	 case 2400:
																					buff_2400[3]=read_air_speed(1);//0x0C;//DMA_Rece_Buf[3];
																					buff_2400[4]=channel_temp;//DMA_Rece_Buf[4];
																					UART_Send((u8*)buff_2400,6);																 
																		 break;
																	 case 4800:
																					 buff_4800[3]=read_air_speed(2);//0x14;//DMA_Rece_Buf[3];
																					 buff_4800[4]=channel_temp;//DMA_Rece_Buf[4];
																					 UART_Send((u8*)buff_4800,6);	
																		 break;
																	 case 9600: //9600--011--3                          
																					 buff_9600[3]=read_air_speed(3);//DMA_Rece_Buf[3];
																					 buff_9600[4]=channel_temp;//0x17;//read_channel();//0x17;//DMA_Rece_Buf[4];
																					 UART_Send((u8*)buff_9600,6);																	          
																	   break;
																	 case 19200:
																		       buff_19200[3]=read_air_speed(4);//DMA_Rece_Buf[3];
																		       buff_19200[4]=channel_temp;//DMA_Rece_Buf[4];
																	         UART_Send((u8*)buff_19200,6);
																	   break;
																	 default:break;																 																 
																 }														
														  LED_TX_ON;//
														 // printf("%s",str);
														 //printf("%d\n",temp_buff[1]);
														 break;                            										 
														 default:
														 break;
													 }
											 break;
								 default:break;				 
							  }
					 break;
			case 0xC2://���öϵ粻����
				        switch(DMA_Rece_Buf[3])
							  {
									case 0x18://---292Bit/s
								  set_air_speed(5);	
									//printf("\r\n  has been modfied to be 9600bps-292bit/s \r\n");							
									break;
									case 0x19://--1300bit/s
								  set_air_speed(1);
                //  printf("\r\n  has been modfied to be 9600bps-1300bit/s \r\n");																	
									break;
									case 0x20:
									set_air_speed(5);//--292bit/s--19200
								//	printf("\r\n  has been modfied to be 19200bps-292bit/s \r\n");
									
									break;
									case 0x21:
								  set_air_speed(1);//--1300bit/s--19200
								//	printf("\r\n  has been modfied to be 19200bps-1300bit/s \r\n");								
									break;
                  case 0x22:
               		set_air_speed(9);//--5468bit/s--19200		
               //   printf("\r\n  has been modfied to be 19200bps-5468bit/s \r\n");																
									break;
									case 0x08:
									set_air_speed(10);//8-1-9
									break;
									case 0x09:
									set_air_speed(11);//9-1-9	
									break;
									case 0x10:
								  set_air_speed(12);//10-1-9	
									break;
									set_air_speed(11);//9-1-9	
									
									default:break;
								}
				   break;
		  case 0xC3://�鿴����汾
				        switch(DMA_Rece_Buf[1])
								{
									case 0xC3:
										 switch(DMA_Rece_Buf[2])
										 {
										   case 0xC3:
											  LED_TX_OFF;//
												// printf("\r\nAS62-T30-V1.0\r\n");
											   //  printf("AS62-T30-V1.0\r\n");											    
												 //delay_ms(800);
											   printf("AS62-T30-V1.0\r\n");	
											   LED_TX_ON;//
											 break;
                       default:break;											 
										 }
									break;
									default:break;
								}
				   break;
			case 0xC4://�����λ
				       switch(DMA_Rece_Buf[2])
							 {
								 case 0xC4:
									    if(DMA_Rece_Buf[3]==0xC4)
											 {NVIC_SystemReset();}  //systerm reset by software
                 break;
                 default:break;											
							 }
				   break;
		  case 0xC5:
				      switch(DMA_Rece_Buf[1])
							{
							  case 0xC5:
									 switch(DMA_Rece_Buf[2])
									 {  
										case 0xC5:
										UART_Send((u8*)C5_buff,3);
										break;
										default:break;
									 }
								break;												
								default:break;
							}
					 break;							 
			default:			
				break;				
		}	
}

/*
 * ��������void bpsCRC(u16 *buff,u8 i)
 * ����  ������bps������
 * ����  ����
 * ���  ����	
 */
void bpsCRC(u16 *buff,u8 i)
{
	u16 tempData;	
	buf[0]=(u8)(buff[i]>>8);
	buf[1]=(u8)(buff[i]&0xff);
	crcData=crc16(buf,2);// ����CRC
	//printf("crcData=%d\n",crcData);
	buf[2]=(u8)(crcData&0xff);
	buf[3]=(u8)(crcData>>8);
	bps_buff[i].buf1=buf[0];//buf[i+1],buf[i+2],buf[i+3]};
	bps_buff[i].buf2=buf[1];
	bps_buff[i].crc1=buf[2];
	bps_buff[i].crc2=buf[3];
	//return  crcData;	
}

/*
 * ��������touchuan_mode(void)
 * ����  ��͸��ģʽ--����תwireless
 * ����  ����
 * ���  ����	
 */
//#elif defined(SX1278_TX)
void touchuan_mode(void)
{ 
   uart_init(crc_flash);//͸����bps 	
	 Usart1_Over=0;
	 LED_RX_OFF;//����LED-TX
	 //checkComm0Modbus();
	 //delay_ms(50); //delay_ms(500);
	// memcpy(temp_buf,DMA_Rece_Buf,Usart1_Rec_Cnt);
	 jieshou_handle();//��λ��->����->����
	 Usart1_Rec_Cnt=0;
	 LED_RX_ON;//�ر�LED-TX	   
}




/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE****/
