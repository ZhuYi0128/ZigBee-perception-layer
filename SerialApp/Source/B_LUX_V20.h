#ifndef __B_LUX_V20_H
#define __B_LUX_V20_H

/*--------------------------头文件引用--------------------------------*/
#include <iocc2530.h>

/*-----------------------------结构体定义---------------------------------*/

/*-----------------------------宏定义---------------------------------*/
//数据类型定义
#define   uint16    unsigned int
#define   uint8     unsigned char
#define   fint32    float
#define   uint32    unsigned long

//引脚定义

#define B_LUX_SCL0_O P0DIR |= 0x01
#define B_LUX_SCL0_H P0_0 = 1
#define B_LUX_SCL0_L P0_0 = 0

#define B_LUX_SDA0_O P0DIR |= 0x40
#define B_LUX_SDA0_H P0_6 = 1
#define B_LUX_SDA0_L P0_6 = 0

#define B_LUX_SDA0_I P1DIR &= ~0x40
#define B_LUX_SDA0_DAT  P0_6

#define	B_LUX_SlaveAddress	  0x46 //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

/*-----------------------------函数声明-------------------------------*/
void B_LUX_delay_nms(uint16 k);
void B_LUX_Init(void);

void  B_LUX_Single_Write(uint8 REG_Address);               //单个写入数据
uint8 B_LUX_Single_Read(uint8 REG_Address);                //单个读取内部寄存器数据
void  B_LUX_Multiple_read(void);                           //连续的读取内部寄存器数据
//------------------------------------
void B_LUX_Delay5us(void);
void B_LUX_Delay5ms(void);
void B_LUX_Start(void);                    //起始信号
void B_LUX_Stop(void);                     //停止信号
void B_LUX_SendACK(uint8 ack);             //应答ACK
uint8  B_LUX_RecvACK(void);                //读ack
void B_LUX_SendByte(uint8 dat);            //IIC单个字节写
uint8 B_LUX_RecvByte(void);                //IIC单个字节读

uint32 B_LUX_GetLux(void);
#endif
