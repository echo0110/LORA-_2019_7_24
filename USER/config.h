#ifndef __config_H
#define	__config_H

#include "stm32f10x.h"
#include  "SX1276.h"
 struct rate
{
 u8 SpreadingFactor;
 u8 CodingRate;
 u8 Bw_Frequency;
};//空中速率结构体块定义

struct bps
{
u8 buf1;
u8 buf2;
u8 crc1;
u8 crc2;
};//波特率+CRC结构体
struct Freq
{
 u8 fre1;
 u8 fre2;
 u8 fre3;
};//频率结构体块定义

//#define   SIZE             sizeof(rate_buff[0])	
extern u16  crc_flash;//从flash里读出来的CRC值
extern u16 crcData;//
extern struct rate  rate_buff[30];//  rate_buff[30];
extern struct bps   bps_buff[10];
extern void config_mode(void);//配置模式声明
extern void bpsCRC(u16 *buff,u8 i);//bps CRC计算
extern void touchuan_mode(void);//透传模式
extern struct Freq  Fre[33];
extern struct rate  datatemp[10];//读取空速时的 临时存储
extern u8 channel_temp;//上电读频段->channel_temp->channel_temp->DMA_Rece_Buf[4]
#endif /* __LED_H */

