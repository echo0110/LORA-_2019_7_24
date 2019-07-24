#ifndef __SX1276__H__
#define __SX1276__H__

#include "stm32f10x.h"
#include "stdio.h"
#define  len     3  
#define  Read_START_ADDR   0x08000000+50*1024
#define  Read_BPS_ADDR     0x08000000+60*1024
#define  Read_air_spped_ADDR    0x08000000+80*1024//0x08070000//0x8000000+100*1024
#define  Read_chnnel_ADDR       0x08000000+100*1024//读信道地址

#define  Read_channel_temp_ADDR   0x08000000+100*1024//设定固定频段 断电保存
#define   SIZE1             sizeof(rate_buff[0])

#define REG_LR_FIFO                                  0x00 
 // Common settings
#define REG_LR_OPMODE                                0x01 
#define REG_LR_BANDSETTING                           0x04
#define REG_LR_FRFMSB                                0x06 
#define REG_LR_FRFMID                                0x07
#define REG_LR_FRFLSB                                0x08 
 // Tx settings
#define REG_LR_PACONFIG                              0x09 
#define REG_LR_PARAMP                                0x0A 
#define REG_LR_OCP                                   0x0B 
 // Rx settings
#define REG_LR_LNA                                   0x0C 
 // LoRa registers
#define REG_LR_FIFOADDRPTR                           0x0D 
#define REG_LR_FIFOTXBASEADDR                        0x0E 
#define REG_LR_FIFORXBASEADDR                        0x0F 
#define REG_LR_FIFORXCURRENTADDR                     0x10 
#define REG_LR_IRQFLAGSMASK                          0x11 
#define REG_LR_IRQFLAGS                              0x12 
#define REG_LR_NBRXBYTES                             0x13 
#define REG_LR_RXHEADERCNTVALUEMSB                   0x14 
#define REG_LR_RXHEADERCNTVALUELSB                   0x15 
#define REG_LR_RXPACKETCNTVALUEMSB                   0x16 
#define REG_LR_RXPACKETCNTVALUELSB                   0x17 
#define REG_LR_MODEMSTAT                             0x18 
#define REG_LR_PKTSNRVALUE                           0x19 
#define REG_LR_PKTRSSIVALUE                          0x1A 
#define REG_LR_RSSIVALUE                             0x1B 
#define REG_LR_HOPCHANNEL                            0x1C 
#define REG_LR_MODEMCONFIG1                          0x1D 
#define REG_LR_MODEMCONFIG2                          0x1E 
#define REG_LR_SYMBTIMEOUTLSB                        0x1F 
#define REG_LR_PREAMBLEMSB                           0x20 
#define REG_LR_PREAMBLELSB                           0x21 
#define REG_LR_PAYLOADLENGTH                         0x22 
#define REG_LR_PAYLOADMAXLENGTH                      0x23 
#define REG_LR_HOPPERIOD                             0x24 
#define REG_LR_FIFORXBYTEADDR                        0x25
#define REG_LR_MODEMCONFIG3                          0x26
 // end of documented register in datasheet
 // I/O settings
#define REG_LR_DIOMAPPING1                           0x40
#define REG_LR_DIOMAPPING2                           0x41
 // Version
#define REG_LR_VERSION                               0x42
 // Additional settings
#define REG_LR_PLLHOP                                0x44
#define REG_LR_TCXO                                  0x4B
#define REG_LR_PADAC                                 0x4D
#define REG_LR_FORMERTEMP                            0x5B
#define REG_LR_BITRATEFRAC                           0x5D
#define REG_LR_AGCREF                                0x61
#define REG_LR_AGCTHRESH1                            0x62
#define REG_LR_AGCTHRESH2                            0x63
#define REG_LR_AGCTHRESH3                            0x64


#define GPIO_VARE_1                                  0X00
#define GPIO_VARE_2                                  0X00
#define RFLR_MODEMCONFIG2_SF_MASK                    0x0f
#define RFLR_MODEMCONFIG1_CODINGRATE_MASK            0xF1 
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK          0xFB 
#define RFLR_MODEMCONFIG1_BW_MASK                    0x0F 
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK        0xFE 
#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK        0xfc
#define RFLR_MODEMCONFIG3_MOBILE_NODE_MASK           0xF7 

#define TIME_OUT_INT                                 0x80 
#define PACKET_RECVER_INT                            0x40 
#define CRC_ERROR_INT                                0x20 
#define RECVER_HEAR_INT                              0x10 
#define FIFO_SEND_OVER                               0x08 
#define RFLR_IRQFLAGS_CAD                            0x04 
#define RFLR_IRQFLAGS_FHSS                           0x02 
#define RFLR_IRQFLAGS_CADD                           0x01 

#define IRQN_TXD_Value                               0xF7
#define IRQN_RXD_Value                               0x9F
#define IRQN_CAD_Value                               0xFA
#define IRQN_SEELP_Value                             0xFF
#define PACKET_MIAX_Value                            0xff


typedef enum
{
    sleep_mode=(unsigned char)0x00,
    stdby_mode=(unsigned char)0x01,
    TX_mode=(unsigned char)0x02,
		Transmitter_mode=(unsigned char)0x03,
    RF_mode=(unsigned char)0x04,
		Recevived_mode=(unsigned char)0x05,
		Recevived_single=(unsigned char)0x06,
		CAD_mode        =(unsigned char)0x07,
}RFMode_SET;
typedef enum
{
  FSK_mode   =(unsigned char)0x00,
	LORA_mode  =(unsigned char)0x80,
}Debugging_fsk_ook;

typedef enum 
{	
	False=0,
	True=1,
}BOOL_t;
typedef enum{enOpen,enClose}cmdCS;//片选CS 控制



//MISO                             
#define  RF_MISO_L	   GPIO_ResetBits(GPIO,GPIO_Pin_7)//
#define  RF_MISO_H     GPIO_SetBits(GPIOA,GPIO_Pin_7)//
//RST
#define  RF_RESET_L	   GPIO_ResetBits(GPIOB,GPIO_Pin_12)//		  
#define  RF_RESET_H	   GPIO_SetBits(GPIOB,GPIO_Pin_12)//
//CS
#define  RF_CS_L	     GPIO_ResetBits(GPIOA,GPIO_Pin_4)//	          
#define  RF_CS_H	     GPIO_SetBits(GPIOA,GPIO_Pin_4)//
//CLK
#define  RF_CKL_L	     GPIO_ResetBits(GPIOA,GPIO_Pin_5)//         
#define  RF_CKL_H	     GPIO_SetBits(GPIOA,GPIO_Pin_5)//
//MOSI---SDI
#define  RF_MOSI_L     	GPIO_ResetBits(GPIOA,GPIO_Pin_7)//           
#define  RF_MOSI_H	    GPIO_SetBits(GPIOA,GPIO_Pin_7)// 
extern void  RF_SPI_INIT(void);
extern void SX1278Reset(void);
extern void RF_SPI_MasterIO(u8 out);//主-->从
extern void SX1276LoRaSetNbTrigPeaks(unsigned char value );
extern void SX1276LoRaSetPacketCrcOn(BOOL_t enable );//CRC打开
//extern void SX1276LoRaSetNbTrigPeaks(unsigned char value );
extern void SX1276LoRaSetMobileNode(BOOL_t enable );
extern void RF_RECEVIVE(void);//连续接收
extern void SX1278Reset(void);//复位
extern  void SX1276LORA_INT(void);//sx1278初始化
extern  void RF_RECEVIVE(void);//sx1278连续接收模式
extern  void SX1276LoRaSetPacketCrcOn(BOOL_t enable );//sx1278 CRC
extern  void sx1278_init(void);//LoRA初始化
extern  void sx1278send(unsigned char dat);//LORA数据发送
extern  void switch_cs(cmdCS  cmd);//片选控制
extern  void  FUN_RF_SENDPACKET(unsigned char *RF_TRAN,unsigned char length);//发射数据
extern  void SX1276WriteData(u8 addr,u8 data);//寄存器写函数
extern  u8 RF_SPI_READ_BYTE(void);//读一字节数据
extern  u8 SX1276ReadData(u8 addr);//寄存器读函数
extern  void	ProcessRecv(void);//读0x12寄存器 触发接收中断
extern  void SX1276LoRaSetOpMode(RFMode_SET opMode );//设置option mode
extern  void SX1276_IO_Init(void);

extern  unsigned char SpreadingFactor;//扩频因子
extern  unsigned char CodingRate;//纠错率
extern  unsigned char Bw_Frequency;//带宽
extern  void set_air_speed(unsigned char i);//设置空中速率
extern  void set_air_speed2();//直接设置空中速率
extern  void Read_flash(void);//读flash值
extern  void SX1276LoRaSetRFchannel(unsigned char k);//set channel
//  rate_buff[30];//空中速率结构体数组

//extern struct rate;


#endif
