#ifndef __config_H
#define	__config_H

#include "stm32f10x.h"
#include  "SX1276.h"
 struct rate
{
 u8 SpreadingFactor;
 u8 CodingRate;
 u8 Bw_Frequency;
};//�������ʽṹ��鶨��

struct bps
{
u8 buf1;
u8 buf2;
u8 crc1;
u8 crc2;
};//������+CRC�ṹ��
struct Freq
{
 u8 fre1;
 u8 fre2;
 u8 fre3;
};//Ƶ�ʽṹ��鶨��

//#define   SIZE             sizeof(rate_buff[0])	
extern u16  crc_flash;//��flash���������CRCֵ
extern u16 crcData;//
extern struct rate  rate_buff[30];//  rate_buff[30];
extern struct bps   bps_buff[10];
extern void config_mode(void);//����ģʽ����
extern void bpsCRC(u16 *buff,u8 i);//bps CRC����
extern void touchuan_mode(void);//͸��ģʽ
extern struct Freq  Fre[33];
extern struct rate  datatemp[10];//��ȡ����ʱ�� ��ʱ�洢
extern u8 channel_temp;//�ϵ��Ƶ��->channel_temp->channel_temp->DMA_Rece_Buf[4]
#endif /* __LED_H */

