//***************************************
// B_LUX_V20采集程序
//****************************************
#include  <math.h>    //Keil library  
#include  <stdio.h>   //Keil library
#include "B_LUX_V20.h"

uint8    BUF_0[8];                       //接收数据缓存区      	
uint16   dis_data_0;                     //变量

/*---------------------------------------------------------------------
 功能描述: 延时纳秒 不同的工作环境,需要调整此函数
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_delay_nms(uint16 k)	
{						
  uint16 i,j;				
  for(i=0;i<k;i++)
  {			
    for(j=0;j<6000;j++)			
    {
      ;
    }
  }						
}					

/*---------------------------------------------------------------------
 功能描述: 延时5微秒  不同的工作环境,需要调整此函数
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Delay5us()
{
  uint8 n = 50;
  
  while (n--);
}

/*---------------------------------------------------------------------
 功能描述: 延时5毫秒  不同的工作环境,需要调整此函数
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Delay5ms()
{
  uint16 n = 50000;
  
  while (n--);
}

/*---------------------------------------------------------------------
 功能描述: 起始信号
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Start()
{
  B_LUX_SDA0_H;                    //拉高数据线
  B_LUX_SCL0_H;                    //拉高时钟线
  B_LUX_Delay5us();                 //延时
  B_LUX_SDA0_L;                    //产生下降沿
  B_LUX_Delay5us();                 //延时
  B_LUX_SCL0_L;                    //拉低时钟线
}

/*---------------------------------------------------------------------
 功能描述: 停止信号
 参数说明: 无	
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Stop()
{
  B_LUX_SDA0_L;                    //拉低数据线
  B_LUX_SCL0_H;                    //拉高时钟线
  B_LUX_Delay5us();                 //延时
  B_LUX_SDA0_H;                    //产生上升沿
  B_LUX_Delay5us();                 //延时
  B_LUX_SCL0_L;
  B_LUX_Delay5us();
}

/*---------------------------------------------------------------------
 功能描述: 发送应答信号
 参数说明: ack - 应答信号(0:ACK 1:NAK)
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_SendACK(uint8 ack)
{
  if (ack&0x01)	B_LUX_SDA0_H;		 //写应答信号
  else	B_LUX_SDA0_L;
  
  B_LUX_SCL0_H;                    //拉高时钟线
  B_LUX_Delay5us();                 //延时
  B_LUX_SCL0_L;                    //拉低时钟线
  B_LUX_SDA0_H;
  B_LUX_Delay5us();                 //延时
}

/*---------------------------------------------------------------------
 功能描述: 接收应答信号
 参数说明: 无
 函数返回: 返回应答信号
 ---------------------------------------------------------------------*/
uint8 B_LUX_RecvACK()
{
  uint8 CY = 0x00;
  B_LUX_SDA0_H;
  
  B_LUX_SDA0_I;
  
  B_LUX_SCL0_H;              //拉高时钟线
  B_LUX_Delay5us();                 //延时
  
  
  CY |= B_LUX_SDA0_DAT;    //读应答信号
  
  B_LUX_Delay5us();                 //延时
  
  B_LUX_SCL0_L;              //拉低时钟线
  
  B_LUX_SDA0_O;
  
  return CY;
}

/*---------------------------------------------------------------------
 功能描述: 向IIC总线发送一个字节数据
 参数说明: dat - 写字节
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_SendByte(uint8 dat)
{
  uint8 i;
  
  for (i=0; i<8; i++)         			//8位计数器
  {
    if (dat&0x80)	B_LUX_SDA0_H;
    else	B_LUX_SDA0_L;              //送数据口
    
    B_LUX_Delay5us();             			//延时
    B_LUX_SCL0_H;                		//拉高时钟线
    B_LUX_Delay5us();             			//延时
    B_LUX_SCL0_L;                		//拉低时钟线
    B_LUX_Delay5us();             			//延时
    dat <<= 1;              			//移出数据的最高位
  }
  
  B_LUX_RecvACK();
}

/*---------------------------------------------------------------------
 功能描述: 从IIC总线接收一个字节数据
 参数说明: 无
 函数返回: 接收字节
 ---------------------------------------------------------------------*/
uint8 B_LUX_RecvByte()
{
  uint8 i;
  uint8 dat = 0;
  B_LUX_SDA0_I;
  
  B_LUX_SDA0_H;                  //使能内部上拉,准备读取数据,
  for (i=0; i<8; i++)         	//8位计数器
  {
    B_LUX_SCL0_H;              //拉高时钟线
    B_LUX_Delay5us();             	//延时
    dat |= B_LUX_SDA0_DAT;     //读数据               
    B_LUX_SCL0_L;              //拉低时钟线
    B_LUX_Delay5us();             	//延时
    
    dat <<= 1;	
  }
  B_LUX_SDA0_O;
  
  return dat;
}

/*---------------------------------------------------------------------
 功能描述: 写BH1750
 参数说明: REG_Address - 寄存器地址
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Single_Write(uint8 REG_Address)
{
  B_LUX_Start();                  //起始信号
  B_LUX_SendByte(B_LUX_SlaveAddress);   //发送设备地址+写信号
  B_LUX_SendByte(REG_Address);    //内部寄存器地址
  B_LUX_Stop();                   //发送停止信号
}

/*---------------------------------------------------------------------
 功能描述: 连续读出BH1750内部数据
 参数说明: 无
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Multiple_read(void)
{   
  uint8 i;	
  B_LUX_Start();                          //起始信号
  B_LUX_SendByte(B_LUX_SlaveAddress+1);         //发送设备地址+读信号
  
  for (i=0; i<3; i++)                        //连续读取6个地址数据，存储中BUF
  {
    BUF_0[i] = B_LUX_RecvByte();          //BUF[0]存储0x32地址中的数据
    if (i == 3)
    {
      
      B_LUX_SendACK(1);                   //最后一个数据需要回NOACK
    }
    else
    {		
      B_LUX_SendACK(0);                   //回应ACK
    }
  }
  
  B_LUX_Stop();                           //停止信号
  B_LUX_Delay5ms();
}

/*---------------------------------------------------------------------
 功能描述: 初始化光照传感器
 参数说明: 无
 函数返回: 无
 ---------------------------------------------------------------------*/
void B_LUX_Init()
{
  
  P1SEL &= ~(0x48);
  
  B_LUX_SCL0_O;
  B_LUX_SDA0_O;
  
  B_LUX_delay_nms(100);	    //延时100ms
  
  B_LUX_Single_Write(0x01); 
  
}

/*---------------------------------------------------------------------
 功能描述: 光照读取函数
 参数说明: 无
 函数返回: 返回光照值
 ---------------------------------------------------------------------*/
uint32 B_LUX_GetLux()
{  
  fint32 temp;
  B_LUX_Single_Write(0x01);   // power on
  B_LUX_Single_Write(0x10);   // H- resolution mode 
  
  B_LUX_delay_nms(180);       //延时180ms
  
  B_LUX_Multiple_read();      //连续读出数据，存储在BUF中
  
  B_LUX_Single_Write(0x00);   // power off
  
  dis_data_0=BUF_0[0];
  dis_data_0=(dis_data_0<<8)+BUF_0[1];//合成数据，即光照数据
  
  temp=(float)dis_data_0/1.2;
  return (uint32)(temp*1.4);
} 

