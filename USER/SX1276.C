#include <stdio.h>
#include "stm32f10x.h"
#include "bsp_usart.h"
#include  "103.h"
#include  "SX1276.h"
#include "stmflash.h"
#include  "led.h"
#include  "usart.h"
#include  "spi.h"
#include  "delay.h"
#include  "config.h"
 
u8 flash_buff[80];//存上电读flash的数据
unsigned char Frequency[3]={0x6c,0x80,0x00};
unsigned char Frequency_410[3]={0x66,0x80,0x11};
unsigned char SpreadingFactor1=7;  //11;//7-12 扩频因子选小一些 ，发射时间会快一些
unsigned char CodingRate1=1;//  4/cr+4  1-4   1--4/5  2--4/6 3--4/7 4--4/8
unsigned char powerValue=0xff;//功率设置
unsigned char Bw_Frequency1=9;
unsigned char sendbuffer[len];
unsigned char Flag;//第一次写falsh后置1
u16  crcData_read;//读出来算的CRC
u16  crc_flash;//从flash里读出来的CRC值
u16  bps;//串口波特率

void SX1276_IO_Init(void)  
{
 SPI_InitTypeDef  SPI_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
//  //打开PA口时钟

//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	GPIO_SetBits(GPIOC,GPIO_Pin_4);
	
	SPI1_Init();    		//初始化SPI	
	SPI_Cmd(SPI1, DISABLE); // 
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							//选择了串行时钟的稳态:时钟悬空低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						//数据捕获于第一个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;							//CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);  								//根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	SPI1_SetSpeed(SPI_BaudRatePrescaler_8);
	SPI_Cmd(SPI1, ENABLE);
}

void  RF_SPI_INIT(void)
{
 RF_CKL_L	;
 RF_CS_H;
 RF_MOSI_H;
}

void SX1278Reset(void)
{
  RF_RESET_L;
	delay_ms(10);//1ms
	RF_RESET_H;
	delay_ms(10);
  SX1276LORA_INT();//sx1278初始化
}

void RF_SPI_MasterIO(u8 out)  /*数据流方向  主-->从机*/  //spi写单个数据  单bit写入 高位在前 依次左移   
{
  unsigned char  i;
	for(i=0;i<8;i++)
	{
	  if(out&0x80)   /*check if MSB is high*/
			RF_MOSI_H;   
		else 
			RF_MOSI_L;
		RF_CKL_H;
		delay_ms(0);
		out=(out<<1);/*shift 1  plce for next bit*/
		RF_CKL_L;
		delay_ms(0);
	}
	//delay_ms(0);
}

	
u8 RF_SPI_READ_BYTE(void) /*数据流方向 从-->主*/
{
 u8 i,j;
	j=0;
	for(i=0;i<8;i++)
	{
	 RF_CKL_H;
		delay(1);//4
	 j=(j<<1);
   if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6))   //MISO
    j=j|0x01;
	 RF_CKL_L;
	 delay(1);//4
	}
	return j;
}
/*
寄存器写函数
*/

 void SX1276WriteData(u8 addr,u8 data)  /*写*/
{
  RF_CS_L;
  SPI1_ReadWriteByte(addr|0x80);/*write addr*/
	SPI1_ReadWriteByte(data);        /*write data*/
  RF_CS_H;

}
/*
寄存器读函数
*/
u8 SX1276ReadData(unsigned char addr)
{
	u16 value;
	RF_CS_L;
	SPI1_ReadWriteByte(addr&0x7f);/*最高位置0 读操作 其他位不变*/
	value=SPI1_ReadWriteByte(0xff);
	RF_CS_H;///NSS=1
	return value;
}
/*
设置option  mode
*/
void SX1276LoRaSetOpMode(RFMode_SET opMode )
{
	unsigned char opModePrev;
	opModePrev=SX1276ReadData(REG_LR_OPMODE);     //读0x01 模式寄存器
	opModePrev &=0xf8;                              //?????
	opModePrev |= (unsigned char)opMode;          //????
	SX1276WriteData(REG_LR_OPMODE, opModePrev); //?????	
}
/*
fsk
*/
void SX1276LoRaFsk(Debugging_fsk_ook opMode )
{
	unsigned char opModePrev;
	opModePrev=SX1276ReadData(REG_LR_OPMODE); //读0x01模式寄存器
	opModePrev &=0x7F; //?????
	opModePrev |= (unsigned char)opMode;  //????
	SX1276WriteData( REG_LR_OPMODE, opModePrev); //?????		
}
/*
set frequency
*/
void SX1276LoRaSetRFFrequency(void)
{
	SX1276WriteData( REG_LR_FRFMSB, Frequency[0]);  //?0x06???
	SX1276WriteData( REG_LR_FRFMID, Frequency[1]);  //?0x07???
	SX1276WriteData( REG_LR_FRFLSB, Frequency[2]);  //?0x08???
}
/*
set channel
*/
void SX1276LoRaSetRFchannel(unsigned char k)
{
	SX1276WriteData( REG_LR_FRFMSB, Fre[k].fre1);  //?0x06???
	SX1276WriteData( REG_LR_FRFMID, Fre[k].fre2);  //?0x07???
	SX1276WriteData( REG_LR_FRFLSB, Fre[k].fre3);  //?0x08???
}



void SX1276LoRaSetRFPower(unsigned char power )
{
	//Set Pmax to +20dBm for PA_HP, Must turn off PA_LF or PA_HF, and set RegOcp
	SX1276WriteData( REG_LR_PACONFIG,  power ); //??????,??????????,???????????
	SX1276WriteData( REG_LR_OCP, 0x3f);  //add by skay,20160810, ????????,
	SX1276WriteData( REG_LR_PADAC, 0x87);  //high power
	//SX1276WriteData( REG_LR_PACONFIG,  power_data[power] ); //用于频谱仪测信号
}


void SX1276LoRaSetSpreadingFactor(unsigned char factor )
{
	unsigned char RECVER_DAT;
	SX1276LoRaSetNbTrigPeaks(3); //0x03-->SF7 to SF12
	RECVER_DAT=SX1276ReadData( REG_LR_MODEMCONFIG2); //读0x1E寄存器  
	RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG2_SF_MASK ) | ( factor << 4 );
	SX1276WriteData( REG_LR_MODEMCONFIG2, RECVER_DAT );	 
}
	
void SX1276LoRaSetNbTrigPeaks(unsigned char value )
{
	unsigned char RECVER_DAT;
	RECVER_DAT = SX1276ReadData(0x31);  //Read RegDetectOptimize,
	RECVER_DAT = ( RECVER_DAT & 0xF8 ) | value; //process;
	SX1276WriteData(0x31, RECVER_DAT );  //write back;
}
	
void SX1276LoRaSetErrorCoding(unsigned char value )
{	
	unsigned char RECVER_DAT;
	RECVER_DAT=SX1276ReadData( REG_LR_MODEMCONFIG1); //读0x1D寄存器
	RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 1 );       //纠错编码率,3~1
	SX1276WriteData( REG_LR_MODEMCONFIG1, RECVER_DAT);
}
	

	
void SX1276LoRaSetSignalBandwidth(unsigned char bw )
{
	unsigned char RECVER_DAT;
	RECVER_DAT=SX1276ReadData( REG_LR_MODEMCONFIG1);  //?0x1D???
	RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG1_BW_MASK ) | ( bw << 4 );
	SX1276WriteData( REG_LR_MODEMCONFIG1, RECVER_DAT );
}

void SX1276LoRaSetPacketCrcOn(BOOL_t enable )
{	
	unsigned char RECVER_DAT;
	RECVER_DAT= SX1276ReadData( REG_LR_MODEMCONFIG2);  //?0x1E??? 
	RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK ) | ( enable << 2 );
	SX1276WriteData( REG_LR_MODEMCONFIG2, RECVER_DAT );
}
	
void SX1276LoRaSetImplicitHeaderOn(BOOL_t enable )
{
	unsigned char RECVER_DAT;
	RECVER_DAT=SX1276ReadData( REG_LR_MODEMCONFIG1 );  //?0x1D???
	RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) | ( enable );
	SX1276WriteData( REG_LR_MODEMCONFIG1, RECVER_DAT );
}
	
void SX1276LoRaSetSymbTimeout(unsigned int value )
{
	unsigned char RECVER_DAT[2];
	RECVER_DAT[0]=SX1276ReadData( REG_LR_MODEMCONFIG2 );    //?0x1E???
	RECVER_DAT[1]=SX1276ReadData( REG_LR_SYMBTIMEOUTLSB );  //?0x1F???
	RECVER_DAT[0] = ( RECVER_DAT[0] & RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) | ( ( value >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK );
	RECVER_DAT[1] = value & 0xFF;
	SX1276WriteData( REG_LR_MODEMCONFIG2, RECVER_DAT[0]);
	SX1276WriteData( REG_LR_SYMBTIMEOUTLSB, RECVER_DAT[1]);
}
	
void SX1276LoRaSetPayloadLength(unsigned char value )
{
	SX1276WriteData( REG_LR_PAYLOADLENGTH, value );  //?0x22???,RegPayloadLength
} 

void SX1276LoRaSetMobileNode(BOOL_t enable )
{	 
	unsigned char RECVER_DAT;
	RECVER_DAT=SX1276ReadData( REG_LR_MODEMCONFIG3 );  //?0x26???,??????
	RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG3_MOBILE_NODE_MASK ) | ( enable << 3 );
	SX1276WriteData( REG_LR_MODEMCONFIG3, RECVER_DAT );
}

void SX1276LORA_INT(void)
{
	struct rate y;
	y.SpreadingFactor=7;
	y.CodingRate=1;
	y.Bw_Frequency=9;
	SX1276LoRaSetOpMode(sleep_mode);        //设置睡眠模式
	SX1276LoRaFsk(LORA_mode);	        // 设置扩频模式
	SX1276LoRaSetOpMode(stdby_mode);        // 设置普通模式
	SX1276WriteData(REG_LR_DIOMAPPING1,GPIO_VARE_1); //写0x40寄存器,DIO引脚映射,??00.
	SX1276WriteData(REG_LR_DIOMAPPING2,GPIO_VARE_2); //写0x41寄存器
	SX1276LoRaSetRFFrequency();                         //频率设置
	SX1276LoRaSetRFPower(powerValue);                   //功率设置
	SX1276LoRaSetSpreadingFactor(rate_buff[0].SpreadingFactor);	    // 扩频因子
	SX1276LoRaSetErrorCoding(rate_buff[0].CodingRate);		    //纠错编码率
	SX1276LoRaSetPacketCrcOn(True);			    //CRC校验打开
	SX1276LoRaSetSignalBandwidth(rate_buff[0].Bw_Frequency);	    //设置频带带宽, 125khz
	SX1276LoRaSetImplicitHeaderOn(False);		    //????????
	SX1276LoRaSetPayloadLength(0xff);      //????????
//	SX1276WriteData( REG_LR_PREAMBLEMSB, 0X00);
//	SX1276WriteData( REG_LR_PREAMBLEMSB, 0X08);
	SX1276LoRaSetSymbTimeout(0x3FF );      //????????,TimeOut = SymbTimeout * Ts?
	SX1276LoRaSetMobileNode(True); 	// true是低速率使用,????LowDataRateOptimize???LoRa???????
  RF_RECEVIVE();           //设置成连续接受模式
}
#if   1
void RF_RECEVIVE(void)
{
	SX1276LoRaSetOpMode(stdby_mode );
	SX1276WriteData(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);	//??CAD???CAD????
	SX1276WriteData(REG_LR_HOPPERIOD,	PACKET_MIAX_Value);
	SX1276WriteData( REG_LR_DIOMAPPING1, 0X00 );
  SX1276WriteData( REG_LR_DIOMAPPING2, 0X00 );		
	SX1276LoRaSetOpMode(Recevived_mode);//设置为连续接收模式

}
#endif

/*
sx1278初始化
*/
void sx1278_init(void)
{
	u16 x1=0x01,x2=0x02,x3=0x03;	
	air_Read_flash();//上电读空速
 	STMFLASH_Read(Read_BPS_ADDR,(u16*)flash_buff,4); //上电读flash数据重新赋值给rate_buff
	crc_flash=flash_buff[1]+(flash_buff[0]<<8);
	 uart_init(crc_flash);//初始化bps为读出的flash的值 //---
	 printf("%d\n",flash_buff[1]+(flash_buff[0]<<8));
	 printf("%d\n",flash_buff[2]);//低8位
	 printf("%d\n",flash_buff[3]);//高8位
  crcData_read=crc16(flash_buff,2);// 读出来数据算的CRC
	
	 
	 printf("crcData_read=%d\n",crcData_read);
	if((flash_buff[2]+(flash_buff[3]<<8))==crcData_read) //判断校验是不是正确
	{ 
	 printf("\r\n CRC check is right\r\n");			
	}
	else
	{
	 printf("\r\n CRC check is wrong\r\n");
	}
	// printf("%d\n",flash_buff[0]);
	//printf("%d\n",flash_buff[1]);
//	rate_buff[0].SpreadingFactor=7;//(u8)(flash_buff[0]);
//	rate_buff[0].CodingRate=1;//(u8)(flash_buff[1]);
//  rate_buff[0].Bw_Frequency=9;//(u8)(flash_buff[2]);	
  SX1278Reset();//sx1278复位 及初始化
	delay_ms(500);
	delay_ms(500);
}
/*
LORA数据发送
*/
void sx1278send(unsigned char dat)
{
 unsigned char i;
 unsigned  Irq_flag=0;
	
	
	
	   FUN_RF_SENDPACKET(DMA_Rece_Buf,Usart1_Rec_Cnt); //发射数据
		Irq_flag=SX1276ReadData(REG_LR_IRQFLAGS ); 
		 while((Irq_flag&0x08) != 0x08)  //TX发送完成后设置该位
    {
      //delay_ms(1);
      Irq_flag=SX1276ReadData( REG_LR_IRQFLAGS ); 
    }

		
	
}
/*
发射模式
*/
void  FUN_RF_SENDPACKET(unsigned char *RF_TRAN,unsigned char length)
{
	unsigned char i;
  SX1276LoRaSetOpMode(stdby_mode);
	SX1276WriteData( REG_LR_HOPPERIOD, 0 );	        //??????
	SX1276WriteData(REG_LR_IRQFLAGSMASK,IRQN_TXD_Value);	//??????
	SX1276WriteData(REG_LR_PAYLOADLENGTH,length);	        //???????
	SX1276WriteData( REG_LR_FIFOTXBASEADDR, 0);           //?Tx FIFO??
	SX1276WriteData( REG_LR_FIFOADDRPTR, 0 );     //SPI interface address pointer in FIFO data buffer
	switch_cs(enOpen);   //开片选
	SPI1_ReadWriteByte(0x80);//SPI突发访问写
	for(i=0;i<length;i++)  //往缓存区里丢数据
	{
	  SPI1_ReadWriteByte(*RF_TRAN);
		RF_TRAN++;
	}
	switch_cs(enClose);//关片选
	SX1276WriteData(REG_LR_DIOMAPPING1,0x40);  //??0x40????0100 0000b,????????????DIO0??
	SX1276WriteData(REG_LR_DIOMAPPING2,0x00);
	SX1276LoRaSetOpMode( Transmitter_mode );     //???????
//	SX1276WriteData( REG_LR_FIFOTXBASEADDR, 0);           //?Tx FIFO??
//	SX1276WriteData( REG_LR_FIFOADDRPTR, 0 ); 
}

//SX1276LORA_INT(void)
/*
片选控制
*/

void switch_cs(cmdCS  cmd)
{
 switch(cmd)
 {
	 case enOpen:
	 {
	   RF_CS_L;
		 break;
	 }
	 case enClose:
	 {
		 RF_CS_H;
	   break;
   }
	 default:
		  break;
 }
}

/*
读0x12寄存器  接收到数据  触发中断
*/

void ProcessRecv(void)
{
    if(SX1276ReadData(REG_LR_IRQFLAGS))  //读0x12寄存器
	   {  			
//			 TIM2->CR1|=TIM_CR1_CEN;//开定时器	
//       Timer2_Init(5-1, 7200-1);//;//以10KHZ的频率计数 计数到5000为500ms  //进去后要关下定时器
        
	   }
}
/*
 * 函数名void set_air_speed(struct rate *p)
 * 描述  ：设置空中速率
 * 输入  ：无
 * 输出  ：无	
 */
void set_air_speed(unsigned char i)
{
	struct rate *p;
//	rate_buff[0].SpreadingFactor=rate_buff[0+i].SpreadingFactor;
//	rate_buff[0].CodingRate=rate_buff[0+i].CodingRate;
//	rate_buff[0].Bw_Frequency=rate_buff[0+i].Bw_Frequency;
  SX1276LoRaSetSpreadingFactor(rate_buff[i].SpreadingFactor);	    // 扩频因子
	SX1276LoRaSetErrorCoding(rate_buff[i].CodingRate);		    //纠错编码率
	SX1276LoRaSetErrorCoding(rate_buff[i].Bw_Frequency);		    //纠错编码率
}


/*
 * 函数名void set_air_speed2(struct rate *p)
 * 描述  ：直接设置空中速率
 * 输入  ：无
 * 输出  ：无	
 */
void set_air_speed2()
{
	rate_buff[0].SpreadingFactor=DMA_Rece_Buf[1];
	rate_buff[0].CodingRate=DMA_Rece_Buf[2];
	rate_buff[0].Bw_Frequency=DMA_Rece_Buf[3];
  SX1276LoRaSetSpreadingFactor(rate_buff[0].SpreadingFactor);	    // 扩频因子
	SX1276LoRaSetErrorCoding(rate_buff[0].CodingRate);		    //纠错编码率
	SX1276LoRaSetErrorCoding(rate_buff[0].Bw_Frequency);		    //纠错编码率
}

/*
 * 函数名：void Read_flash(void)
 * 描述  ：读Flash值
 * 输入  ：无
 * 输出  ：无	
 */
void Read_flash(void)
{
 STMFLASH_Read(Read_BPS_ADDR,(u16*)flash_buff,4); //上电读flash数据重新赋值给rate_buff
	// uart_init(flash_buff[1]+(flash_buff[0]<<8));//上电设波特率 
 crc_flash=flash_buff[1]+(flash_buff[0]<<8);
}




