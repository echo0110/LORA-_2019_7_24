/*************************************************************************************************
                                          
*************************************************************************************************/

#ifndef __103_h
#define __103_h

#include "stm32f10x.h"



extern  void UART_Send(u8 *str,u8 len3);//����4����
extern  void jieshou_handle(void);//�ɱ�֡���մ���
extern  void checkComm0Modbus(void);//CRCУ��
extern  void GetRxPacket(void);//���߽���
extern  u16   crc16(u8 *puchMsg, u16 usDataLen);//��CRC
extern  void air_Read_flash(void);//��flashֵ
extern  u8 read_air_speed(unsigned char bps);//��flash���  ����ֵ
extern  u8 read_channel(void);//��flash���  �ŵ�ֵ
extern  u8 read_channel_temp(void);







#endif
