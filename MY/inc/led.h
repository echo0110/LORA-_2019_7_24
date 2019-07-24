#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"

#define LED2_OFF GPIO_SetBits(GPIOE,GPIO_Pin_5)
#define LED2_ON GPIO_ResetBits(GPIOE,GPIO_Pin_5)
#define LED2_REV GPIO_WriteBit(GPIOE, GPIO_Pin_5,(BitAction)(1-(GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_5))))

#define LED3_OFF GPIO_SetBits(GPIOB,GPIO_Pin_5)
#define LED3_ON GPIO_ResetBits(GPIOB,GPIO_Pin_5)
#define LED3_REV GPIO_WriteBit(GPIOB, GPIO_Pin_5,(BitAction)(1-(GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_5))))

#define LED_TX_REV  GPIO_WriteBit(GPIOC, GPIO_Pin_7,(BitAction)(1-(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_7))))
#define LED_RX_REV  GPIO_WriteBit(GPIOC, GPIO_Pin_6,(BitAction)(1-(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_6))))
#define LED_TX_ON   GPIO_SetBits(GPIOC,GPIO_Pin_7)
#define LED_TX_OFF  GPIO_ResetBits(GPIOC,GPIO_Pin_7)

#define LED_RX_ON   GPIO_SetBits(GPIOC,GPIO_Pin_6)
#define LED_RX_OFF  GPIO_ResetBits(GPIOC,GPIO_Pin_6)

#define LED4_OFF GPIO_SetBits(GPIOC,GPIO_Pin_9)
#define LED4_ON  GPIO_ResetBits(GPIOC,GPIO_Pin_9)
#define LED4_REV GPIO_WriteBit(GPIOC, GPIO_Pin_9,(BitAction)(1-(GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_9))))


extern  u8  temp_buf2[255];//存无线接收数据
void    LED_Init(void);
extern  void Timer2_Init(u16 arr, u16 psc);
extern  void Timer3_Init(u16 arr, u16 psc);
extern  void reset_Init(void);//复位引脚初始化
extern  void TIM2_NVIC_Configuration(void);//NVIC配置
extern  void SetTxPacket(void);//无线发射
extern  void TX_RX_LED(void);//LED指示收发
extern  void Key(void);//按键引脚初始化
extern  void led_toggle(void);//led电平翻转

//extern  u16  x5;
extern  unsigned char   RF_EX0_STATUS;//
extern  unsigned char   CRC_Value;//CRC值
extern  unsigned char   x5;
extern  int j;

#endif
/*----------------------德飞莱 技术论坛：www.doflye.net--------------------------*/
