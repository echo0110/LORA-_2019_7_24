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
 
u8 flash_buff[80];//���ϵ��flash������
unsigned char Frequency[3]={0x6c,0x80,0x00};
unsigned char Frequency_410[3]={0x66,0x80,0x11};
unsigned char SpreadingFactor1=7;  //11;//7-12 ��Ƶ����ѡСһЩ ������ʱ����һЩ
unsigned char CodingRate1=1;//  4/cr+4  1-4   1--4/5  2--4/6 3--4/7 4--4/8
unsigned char powerValue=0xff;//��������
unsigned char Bw_Frequency1=9;
unsigned char sendbuffer[len];
unsigned char Flag;//��һ��дfalsh����1
u16  crcData_read;//���������CRC
u16  crc_flash;//��flash���������CRCֵ
u16  bps;//���ڲ�����

void SX1276_IO_Init(void)  
{
 SPI_InitTypeDef  SPI_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
//  //��PA��ʱ��

//
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	GPIO_SetBits(GPIOC,GPIO_Pin_4);
	
	SPI1_Init();    		//��ʼ��SPI	
	SPI_Cmd(SPI1, DISABLE); // 
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							//ѡ���˴���ʱ�ӵ���̬:ʱ�����յ͵�ƽ
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						//���ݲ����ڵ�һ��ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//NSS�ź���Ӳ����NSS�ܽţ�����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 7;							//CRCֵ����Ķ���ʽ
	SPI_Init(SPI1, &SPI_InitStructure);  								//����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
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
  SX1276LORA_INT();//sx1278��ʼ��
}

void RF_SPI_MasterIO(u8 out)  /*����������  ��-->�ӻ�*/  //spiд��������  ��bitд�� ��λ��ǰ ��������   
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

	
u8 RF_SPI_READ_BYTE(void) /*���������� ��-->��*/
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
�Ĵ���д����
*/

 void SX1276WriteData(u8 addr,u8 data)  /*д*/
{
  RF_CS_L;
  SPI1_ReadWriteByte(addr|0x80);/*write addr*/
	SPI1_ReadWriteByte(data);        /*write data*/
  RF_CS_H;

}
/*
�Ĵ���������
*/
u8 SX1276ReadData(unsigned char addr)
{
	u16 value;
	RF_CS_L;
	SPI1_ReadWriteByte(addr&0x7f);/*���λ��0 ������ ����λ����*/
	value=SPI1_ReadWriteByte(0xff);
	RF_CS_H;///NSS=1
	return value;
}
/*
����option  mode
*/
void SX1276LoRaSetOpMode(RFMode_SET opMode )
{
	unsigned char opModePrev;
	opModePrev=SX1276ReadData(REG_LR_OPMODE);     //��0x01 ģʽ�Ĵ���
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
	opModePrev=SX1276ReadData(REG_LR_OPMODE); //��0x01ģʽ�Ĵ���
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
	//SX1276WriteData( REG_LR_PACONFIG,  power_data[power] ); //����Ƶ���ǲ��ź�
}


void SX1276LoRaSetSpreadingFactor(unsigned char factor )
{
	unsigned char RECVER_DAT;
	SX1276LoRaSetNbTrigPeaks(3); //0x03-->SF7 to SF12
	RECVER_DAT=SX1276ReadData( REG_LR_MODEMCONFIG2); //��0x1E�Ĵ���  
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
	RECVER_DAT=SX1276ReadData( REG_LR_MODEMCONFIG1); //��0x1D�Ĵ���
	RECVER_DAT = ( RECVER_DAT & RFLR_MODEMCONFIG1_CODINGRATE_MASK ) | ( value << 1 );       //����������,3~1
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
	SX1276LoRaSetOpMode(sleep_mode);        //����˯��ģʽ
	SX1276LoRaFsk(LORA_mode);	        // ������Ƶģʽ
	SX1276LoRaSetOpMode(stdby_mode);        // ������ͨģʽ
	SX1276WriteData(REG_LR_DIOMAPPING1,GPIO_VARE_1); //д0x40�Ĵ���,DIO����ӳ��,??00.
	SX1276WriteData(REG_LR_DIOMAPPING2,GPIO_VARE_2); //д0x41�Ĵ���
	SX1276LoRaSetRFFrequency();                         //Ƶ������
	SX1276LoRaSetRFPower(powerValue);                   //��������
	SX1276LoRaSetSpreadingFactor(rate_buff[0].SpreadingFactor);	    // ��Ƶ����
	SX1276LoRaSetErrorCoding(rate_buff[0].CodingRate);		    //����������
	SX1276LoRaSetPacketCrcOn(True);			    //CRCУ���
	SX1276LoRaSetSignalBandwidth(rate_buff[0].Bw_Frequency);	    //����Ƶ������, 125khz
	SX1276LoRaSetImplicitHeaderOn(False);		    //????????
	SX1276LoRaSetPayloadLength(0xff);      //????????
//	SX1276WriteData( REG_LR_PREAMBLEMSB, 0X00);
//	SX1276WriteData( REG_LR_PREAMBLEMSB, 0X08);
	SX1276LoRaSetSymbTimeout(0x3FF );      //????????,TimeOut = SymbTimeout * Ts?
	SX1276LoRaSetMobileNode(True); 	// true�ǵ�����ʹ��,????LowDataRateOptimize???LoRa???????
  RF_RECEVIVE();           //���ó���������ģʽ
}
#if   1
void RF_RECEVIVE(void)
{
	SX1276LoRaSetOpMode(stdby_mode );
	SX1276WriteData(REG_LR_IRQFLAGSMASK,IRQN_RXD_Value);	//??CAD???CAD????
	SX1276WriteData(REG_LR_HOPPERIOD,	PACKET_MIAX_Value);
	SX1276WriteData( REG_LR_DIOMAPPING1, 0X00 );
  SX1276WriteData( REG_LR_DIOMAPPING2, 0X00 );		
	SX1276LoRaSetOpMode(Recevived_mode);//����Ϊ��������ģʽ

}
#endif

/*
sx1278��ʼ��
*/
void sx1278_init(void)
{
	u16 x1=0x01,x2=0x02,x3=0x03;	
	air_Read_flash();//�ϵ������
 	STMFLASH_Read(Read_BPS_ADDR,(u16*)flash_buff,4); //�ϵ��flash�������¸�ֵ��rate_buff
	crc_flash=flash_buff[1]+(flash_buff[0]<<8);
	 uart_init(crc_flash);//��ʼ��bpsΪ������flash��ֵ //---
	 printf("%d\n",flash_buff[1]+(flash_buff[0]<<8));
	 printf("%d\n",flash_buff[2]);//��8λ
	 printf("%d\n",flash_buff[3]);//��8λ
  crcData_read=crc16(flash_buff,2);// �������������CRC
	
	 
	 printf("crcData_read=%d\n",crcData_read);
	if((flash_buff[2]+(flash_buff[3]<<8))==crcData_read) //�ж�У���ǲ�����ȷ
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
  SX1278Reset();//sx1278��λ ����ʼ��
	delay_ms(500);
	delay_ms(500);
}
/*
LORA���ݷ���
*/
void sx1278send(unsigned char dat)
{
 unsigned char i;
 unsigned  Irq_flag=0;
	
	
	
	   FUN_RF_SENDPACKET(DMA_Rece_Buf,Usart1_Rec_Cnt); //��������
		Irq_flag=SX1276ReadData(REG_LR_IRQFLAGS ); 
		 while((Irq_flag&0x08) != 0x08)  //TX������ɺ����ø�λ
    {
      //delay_ms(1);
      Irq_flag=SX1276ReadData( REG_LR_IRQFLAGS ); 
    }

		
	
}
/*
����ģʽ
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
	switch_cs(enOpen);   //��Ƭѡ
	SPI1_ReadWriteByte(0x80);//SPIͻ������д
	for(i=0;i<length;i++)  //���������ﶪ����
	{
	  SPI1_ReadWriteByte(*RF_TRAN);
		RF_TRAN++;
	}
	switch_cs(enClose);//��Ƭѡ
	SX1276WriteData(REG_LR_DIOMAPPING1,0x40);  //??0x40????0100 0000b,????????????DIO0??
	SX1276WriteData(REG_LR_DIOMAPPING2,0x00);
	SX1276LoRaSetOpMode( Transmitter_mode );     //???????
//	SX1276WriteData( REG_LR_FIFOTXBASEADDR, 0);           //?Tx FIFO??
//	SX1276WriteData( REG_LR_FIFOADDRPTR, 0 ); 
}

//SX1276LORA_INT(void)
/*
Ƭѡ����
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
��0x12�Ĵ���  ���յ�����  �����ж�
*/

void ProcessRecv(void)
{
    if(SX1276ReadData(REG_LR_IRQFLAGS))  //��0x12�Ĵ���
	   {  			
//			 TIM2->CR1|=TIM_CR1_CEN;//����ʱ��	
//       Timer2_Init(5-1, 7200-1);//;//��10KHZ��Ƶ�ʼ��� ������5000Ϊ500ms  //��ȥ��Ҫ���¶�ʱ��
        
	   }
}
/*
 * �������Svoid set_air_speed(struct rate *p)
 * ����  �����ÿ�������
 * ����  ����
 * ���  ����	
 */
void set_air_speed(unsigned char i)
{
	struct rate *p;
//	rate_buff[0].SpreadingFactor=rate_buff[0+i].SpreadingFactor;
//	rate_buff[0].CodingRate=rate_buff[0+i].CodingRate;
//	rate_buff[0].Bw_Frequency=rate_buff[0+i].Bw_Frequency;
  SX1276LoRaSetSpreadingFactor(rate_buff[i].SpreadingFactor);	    // ��Ƶ����
	SX1276LoRaSetErrorCoding(rate_buff[i].CodingRate);		    //����������
	SX1276LoRaSetErrorCoding(rate_buff[i].Bw_Frequency);		    //����������
}


/*
 * �������Svoid set_air_speed2(struct rate *p)
 * ����  ��ֱ�����ÿ�������
 * ����  ����
 * ���  ����	
 */
void set_air_speed2()
{
	rate_buff[0].SpreadingFactor=DMA_Rece_Buf[1];
	rate_buff[0].CodingRate=DMA_Rece_Buf[2];
	rate_buff[0].Bw_Frequency=DMA_Rece_Buf[3];
  SX1276LoRaSetSpreadingFactor(rate_buff[0].SpreadingFactor);	    // ��Ƶ����
	SX1276LoRaSetErrorCoding(rate_buff[0].CodingRate);		    //����������
	SX1276LoRaSetErrorCoding(rate_buff[0].Bw_Frequency);		    //����������
}

/*
 * ��������void Read_flash(void)
 * ����  ����Flashֵ
 * ����  ����
 * ���  ����	
 */
void Read_flash(void)
{
 STMFLASH_Read(Read_BPS_ADDR,(u16*)flash_buff,4); //�ϵ��flash�������¸�ֵ��rate_buff
	// uart_init(flash_buff[1]+(flash_buff[0]<<8));//�ϵ��貨���� 
 crc_flash=flash_buff[1]+(flash_buff[0]<<8);
}



