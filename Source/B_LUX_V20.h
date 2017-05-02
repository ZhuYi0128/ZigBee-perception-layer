#ifndef __B_LUX_V20_H
#define __B_LUX_V20_H

/*--------------------------ͷ�ļ�����--------------------------------*/
#include <iocc2530.h>

/*-----------------------------�ṹ�嶨��---------------------------------*/

/*-----------------------------�궨��---------------------------------*/
//�������Ͷ���
#define   uint16    unsigned int
#define   uint8     unsigned char
#define   fint32    float
#define   uint32    unsigned long

//���Ŷ���

#define B_LUX_SCL0_O P0DIR |= 0x01
#define B_LUX_SCL0_H P0_0 = 1
#define B_LUX_SCL0_L P0_0 = 0

#define B_LUX_SDA0_O P0DIR |= 0x40
#define B_LUX_SDA0_H P0_6 = 1
#define B_LUX_SDA0_L P0_6 = 0

#define B_LUX_SDA0_I P1DIR &= ~0x40
#define B_LUX_SDA0_DAT  P0_6

#define	B_LUX_SlaveAddress	  0x46 //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�

/*-----------------------------��������-------------------------------*/
void B_LUX_delay_nms(uint16 k);
void B_LUX_Init(void);

void  B_LUX_Single_Write(uint8 REG_Address);               //����д������
uint8 B_LUX_Single_Read(uint8 REG_Address);                //������ȡ�ڲ��Ĵ�������
void  B_LUX_Multiple_read(void);                           //�����Ķ�ȡ�ڲ��Ĵ�������
//------------------------------------
void B_LUX_Delay5us(void);
void B_LUX_Delay5ms(void);
void B_LUX_Start(void);                    //��ʼ�ź�
void B_LUX_Stop(void);                     //ֹͣ�ź�
void B_LUX_SendACK(uint8 ack);             //Ӧ��ACK
uint8  B_LUX_RecvACK(void);                //��ack
void B_LUX_SendByte(uint8 dat);            //IIC�����ֽ�д
uint8 B_LUX_RecvByte(void);                //IIC�����ֽڶ�

uint32 B_LUX_GetLux(void);
#endif
