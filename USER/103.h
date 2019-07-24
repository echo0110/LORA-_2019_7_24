/*************************************************************************************************
                                          
*************************************************************************************************/

#ifndef __103_h
#define __103_h

#include "stm32f10x.h"



extern  void UART_Send(u8 *str,u8 len3);//串口4发送
extern  void jieshou_handle(void);//可变帧接收处理
extern  void checkComm0Modbus(void);//CRC校验
extern  void GetRxPacket(void);//无线接收
extern  u16   crc16(u8 *puchMsg, u16 usDataLen);//算CRC
extern  void air_Read_flash(void);//读flash值
extern  u8 read_air_speed(unsigned char bps);//读flash里的  空速值
extern  u8 read_channel(void);//读flash里的  信道值
extern  u8 read_channel_temp(void);







#endif
