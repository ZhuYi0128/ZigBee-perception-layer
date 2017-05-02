/*********************************************************************
* INCLUDES 
6.19

*/
#include <stdio.h>
#include <string.h>

#include "AF.h"
#include "OnBoard.h"
#include "OSAL_Tasks.h"
#include "SerialApp.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "B_LUX_V20.h"
#include "hal_drivers.h"
#include "hal_key.h"
#if defined ( LCD_SUPPORTED )
#include "hal_lcd.h"
#endif
#include "hal_led.h"
#include "hal_uart.h"

#include "DHT11.h"
#include "nwk_globals.h"

/*********************************************************************
* MACROS
*/
#define COORD_ADDR   0x00
#define ED_ADDR      0x01
#define UART0        0x00
//#define MAX_NODE     0x04
#define UART_DEBUG   0x00        //调试宏,通过串口输出协调器和终端的IEEE、短地址
//#define LAMP_PIN     P0_4        //定义P0.4口为继电器输入端
//#define GAS_PIN      P0_5        //定义P0.5口为烟雾传感器的输入端  

#define LED_PIN      P0_0           //定义P0.3口为植物光LED继电器输入端
#define heat_PIN     P1_0           //定义P0.4口为加热垫继电器输入端
#define Bfan_PIN     P0_4          //定义P0.5口为大风机继电器输入端
#define Sfan_PIN     P0_0          //定义P0.6口为小风机继电器输入端
#define DRIP_PIN     P0_5
#define HUM_PIN      P1_1           //定义P0.2口为土壤湿度2输入端
#define DAMP_PIN     P0_6
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr)[0])

//---------------------------------------------------------------------
//标准版不同的终端需要修改此ID,用于识别协调器发过来的数据，ID相同则处理
//专业版自动从Flash获得地址，所有终端固件相同，适合量产
static uint16 EndDeviceID = 0x01; //终端ID，重要
//---------------------------------------------------------------------

/*********************************************************************
* CONSTANTS
*/

#if !defined( SERIAL_APP_PORT )
#define SERIAL_APP_PORT  0
#endif

#if !defined( SERIAL_APP_BAUD )
#define SERIAL_APP_BAUD  HAL_UART_BR_9600
//#define SERIAL_APP_BAUD  HAL_UART_BR_115200
#endif

// When the Rx buf space is less than this threshold, invoke the Rx callback.
#if !defined( SERIAL_APP_THRESH )
#define SERIAL_APP_THRESH  64
#endif

#if !defined( SERIAL_APP_RX_SZ )
#define SERIAL_APP_RX_SZ  128
#endif

#if !defined( SERIAL_APP_TX_SZ )
#define SERIAL_APP_TX_SZ  128
#endif

// Millisecs of idle time after a byte is received before invoking Rx callback.
#if !defined( SERIAL_APP_IDLE )
#define SERIAL_APP_IDLE  6
#endif

// Loopback Rx bytes to Tx for throughput testing.
#if !defined( SERIAL_APP_LOOPBACK )
#define SERIAL_APP_LOOPBACK  FALSE
#endif

// This is the max byte count per OTA message.
#if !defined( SERIAL_APP_TX_MAX )
#define SERIAL_APP_TX_MAX  20
#endif

#define SERIAL_APP_RSP_CNT  4

// This list should be filled with Application specific Cluster IDs.
const cId_t SerialApp_ClusterList[SERIALAPP_MAX_CLUSTERS] =
{
	SERIALAPP_CLUSTERID
};

const SimpleDescriptionFormat_t SerialApp_SimpleDesc =
{
    SERIALAPP_ENDPOINT,              //  int   Endpoint;                端口号
    SERIALAPP_PROFID,                //  uint16 AppProfId[2];           profile ID
    SERIALAPP_DEVICEID,              //  uint16 AppDeviceId[2];         设备ID
    SERIALAPP_DEVICE_VERSION,        //  int   AppDevVer:4;             版本号
    SERIALAPP_FLAGS,                 //  int   AppFlags:4;              程序标识
    SERIALAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;        输入命令数
    (cId_t *)SerialApp_ClusterList,  //  byte *pAppInClusterList;       输入命令表
    SERIALAPP_MAX_CLUSTERS,          //  byte  AppNumOutClusters;       输出命令数
    (cId_t *)SerialApp_ClusterList   //  byte *pAppOutClusterList;      输出命令表
};

const endPointDesc_t SerialApp_epDesc =                 // 端口描述符
{ 
    SERIALAPP_ENDPOINT,                                 // 端口号
    &SerialApp_TaskID,                                  // 任务ID
    (SimpleDescriptionFormat_t *)&SerialApp_SimpleDesc, // 简单描述符
    noLatencyReqs                                       // 网络启动模式（ noLatencyReqs fastBeacons slowBeacons）
};

/*********************************************************************
* TYPEDEFS
*/

/*********************************************************************
* GLOBAL VARIABLES
*/

uint8 SerialApp_TaskID;    // Task ID for internal task/event processing.

/*********************************************************************
* EXTERNAL VARIABLES
*/

/*********************************************************************
* EXTERNAL FUNCTIONS
*/

/*********************************************************************
* LOCAL VARIABLES
*/
static bool SendFlag = 0;   // 就两个状态 0或者1 

static uint8 SerialApp_MsgID;

static afAddrType_t SerialApp_TxAddr;
static afAddrType_t Broadcast_DstAddr;

static uint8 SerialApp_TxSeq;
static uint8 SerialApp_TxBuf[SERIAL_APP_TX_MAX+1];
static uint8 SerialApp_TxLen;

static afAddrType_t SerialApp_RxAddr;
static uint8 SerialApp_RspBuf[SERIAL_APP_RSP_CNT];
uint8 low_flag;
uint8 high_flag;
uint8 shilow_flag;
uint8 shihigh_flag;
uint8 lux_warn_flag;
uint8 pump_flag;

static devStates_t SerialApp_NwkState;
static afAddrType_t SerialApp_TxAddr;
static uint8 SerialApp_MsgID;

uint8 NodeData1[15];         //终端数据缓冲区 0=温度 1=湿度 2=气体 3=灯  [MAX_NODE]
uint8 NodeData2[15];
uint8 NodeData3[15];
/*********************************************************************
* LOCAL FUNCTIONS
*/

static void SerialApp_HandleKeys( uint8 shift, uint8 keys );
static void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt );
static void SerialApp_Send(void);
static void SerialApp_Resp(void);
static void SerialApp_CallBack(uint8 port, uint8 event);

//static void PrintAddrInfo(uint16 shortAddr, uint8 *pIeeeAddr);
//static void AfSendAddrInfo(void);
//static void GetIeeeAddr(uint8 * pIeeeAddr, uint8 *pStr);
static void SerialApp_SendPeriodicMessage( void );
//static uint8 GetDataLen(uint8 fc);
/*static uint8 GetLamp( void );
static uint8 GetGas( void );*/
static uint8 GetHUM( void );
static uint8 GetLED( void );
static uint8 Getheat( void );
static uint8 GetDRIP( void );
static uint8 GetDAMP( void );
static uint8 GetBfan( void );
static uint8 GetSfan( void );
//static uint8 XorCheckSum(uint8 * pBuf, uint8 len);
uint8 SendData(uint8 AD,uint8 TP);
/*********************************************************************
* @fn      SerialApp_Init
*
* @brief   This is called during OSAL tasks' initialization.
*
* @param   task_id - the Task ID assigned by OSAL.
*
* @return  none
*/
void SerialApp_Init( uint8 task_id )
{
	halUARTCfg_t uartConfig;
    
    /*P0SEL &= 0xEf;                  //设置P0.4口为普通IO
    P0DIR |= 0x10;                  //设置P0.4为输出
    LAMP_PIN = 1;                   //高电平继电器断开;低电平继电器吸合
    P0SEL &= ~0x20;                 //设置P0.5为普通IO口
    P0DIR &= ~0x20;                 //P0.5定义为输入口
    P0SEL &= 0x7f;                  //P0_7配置成通用io*/
    
    //P0SEL &= 0x8e;                  //设置P0.0、0.4、0.5、0.6口为普通IO
    //P0DIR |= 0x71;                  //设置PP0.0、0.4、0.5、0.6为输出
    
    P0SEL &= 0x7f;                  //P0_7配置成通用io
    
    //P0SEL &= 0x3f;    
        
    //P1SEL &= 0xfe;                  //P1.0设置为普通口
    //P1DIR |= 0x01;                  //P1.0设置为输出口
    //P1SEL &= ~0x02;                 //设置P1.1为普通IO口
    //P1DIR &= ~0x02;                 //设置P1.1为输入口
    
    LED_PIN  = 1;
    heat_PIN = 1;
    Bfan_PIN = 0;
    Sfan_PIN = 1;
    DAMP_PIN = 1;
    DRIP_PIN = 1;                  //高电平继电器断开;低电平继电器吸合
	
	SerialApp_TaskID = task_id;
	//SerialApp_RxSeq = 0xC3;
	
	afRegister( (endPointDesc_t *)&SerialApp_epDesc );
	
	RegisterForKeys( task_id );
	
	uartConfig.configured           = TRUE;              // 2x30 don't care - see uart driver.
	uartConfig.baudRate             = SERIAL_APP_BAUD;   // 波特率
	uartConfig.flowControl          = FALSE;             // 流控
	uartConfig.flowControlThreshold = SERIAL_APP_THRESH; // 流控阀值
	uartConfig.rx.maxBufSize        = SERIAL_APP_RX_SZ;  // 最大接收量
	uartConfig.tx.maxBufSize        = SERIAL_APP_TX_SZ;  // 最大发送量
	uartConfig.idleTimeout          = SERIAL_APP_IDLE;   // 2x30 don't care - see uart driver.
	uartConfig.intEnable            = TRUE;              // 2x30 don't care - see uart driver.
	uartConfig.callBackFunc         = SerialApp_CallBack;
	HalUARTOpen (UART0, &uartConfig);
	
//#if defined ( LCD_SUPPORTED )
//	HalLcdWriteString( "SerialApp", HAL_LCD_LINE_2 );
//#endif
	//HalUARTWrite(UART0, "Init", 4);
	//ZDO_RegisterForZDOMsg( SerialApp_TaskID, End_Device_Bind_rsp );
	//ZDO_RegisterForZDOMsg( SerialApp_TaskID, Match_Desc_rsp );
}

/*********************************************************************
* @fn      SerialApp_ProcessEvent
*
* @brief   Generic Application Task event processor.
*
* @param   task_id  - The OSAL assigned task ID.
* @param   events   - Bit map of events to process.
*
* @return  Event flags of all unprocessed events.
*/
UINT16 SerialApp_ProcessEvent( uint8 task_id, UINT16 events )
{
	(void)task_id;  // Intentionally unreferenced parameter
	
	if ( events & SYS_EVENT_MSG )
	{
		afIncomingMSGPacket_t *MSGpkt;
		
		while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SerialApp_TaskID )) )
		{
			switch ( MSGpkt->hdr.event )
			{
			case ZDO_CB_MSG:
				//SerialApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
				break;
				
			case KEY_CHANGE:
				SerialApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
				break;
				
			case AF_INCOMING_MSG_CMD:  // 接受到命令 然后执行
				SerialApp_ProcessMSGCmd( MSGpkt ); // 执行进来的消息命令的回调函数
				break;
                
            case ZDO_STATE_CHANGE:
              SerialApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
              if ( (SerialApp_NwkState == DEV_ZB_COORD)
                  || (SerialApp_NwkState == DEV_ROUTER)
                  || (SerialApp_NwkState == DEV_END_DEVICE) )
              {
                #if defined(ZDO_COORDINATOR) //协调器通过串口输出自身短地址、IEEE  
                    Broadcast_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
                    Broadcast_DstAddr.endPoint = SERIALAPP_ENDPOINT;
                    Broadcast_DstAddr.addr.shortAddr = 0xFFFF;
                    #if UART_DEBUG           
                    PrintAddrInfo( NLME_GetShortAddr(), aExtendedAddress + Z_EXTADDR_LEN - 1);
                    #endif 
                    //初始化灯的状态，1为熄灭状态，0为点亮
                    //NodeData[3] = 1;
                    /*NodeData[1][3] = 1;
                    NodeData[2][3] = 1;
                    NodeData[3][3] = 1;*/
                #else                        //终端无线发送短地址、IEEE   
                    //AfSendAddrInfo();
                #endif
                
              }
              break;				
			default:
				break;
			}
			
			osal_msg_deallocate( (uint8 *)MSGpkt );
		}
		
		return ( events ^ SYS_EVENT_MSG );
	}
    
    //在此事件中可以定时向协调器发送节点传感器参数信息
    if ( events & SERIALAPP_SEND_PERIODIC_EVT )
    {
        SerialApp_SendPeriodicMessage();
        
        osal_start_timerEx( SerialApp_TaskID, SERIALAPP_SEND_PERIODIC_EVT,
            (SERIALAPP_SEND_PERIODIC_TIMEOUT + (osal_rand() & 0x00FF)) );
        
        return (events ^ SERIALAPP_SEND_PERIODIC_EVT);
    }
    
	if ( events & SERIALAPP_SEND_EVT )
	{
		SerialApp_Send();
		return ( events ^ SERIALAPP_SEND_EVT );
	}
	
	if ( events & SERIALAPP_RESP_EVT )
	{
		SerialApp_Resp();
		return ( events ^ SERIALAPP_RESP_EVT );
	}
	
	return ( 0 ); 
}

/*********************************************************************
* @fn      SerialApp_HandleKeys
*
* @brief   Handles all key events for this device.
*
* @param   shift - true if in shift/alt.
* @param   keys  - bit field for key events.
*
* @return  none
*/
void SerialApp_HandleKeys( uint8 shift, uint8 keys )
{
	zAddrType_t txAddr;
	
    if ( keys & HAL_KEY_SW_6 ) //按S1键启动或停止终端定时上报数据 
    {
      if(SendFlag == 0)
        {
        SendFlag = 1;
        HalLedSet ( HAL_LED_1, HAL_LED_MODE_ON );
        osal_start_timerEx( SerialApp_TaskID,
                            SERIALAPP_SEND_PERIODIC_EVT,
                            SERIALAPP_SEND_PERIODIC_TIMEOUT );
        }
        else
        {      
            SendFlag = 0;
            HalLedSet ( HAL_LED_1, HAL_LED_MODE_OFF );
            osal_stop_timerEx(SerialApp_TaskID, SERIALAPP_SEND_PERIODIC_EVT);
        }
    }
    
    if ( keys & HAL_KEY_SW_1 ) //按S2
    {
        //LAMP_PIN = ~LAMP_PIN;
    }
    
    if ( keys & HAL_KEY_SW_2 )
    {
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
        
        // Initiate an End Device Bind Request for the mandatory endpoint
        txAddr.addrMode = Addr16Bit;
        txAddr.addr.shortAddr = 0x0000; // Coordinator
        ZDP_EndDeviceBindReq( &txAddr, NLME_GetShortAddr(), 
            SerialApp_epDesc.endPoint,
            SERIALAPP_PROFID,
            SERIALAPP_MAX_CLUSTERS, (cId_t *)SerialApp_ClusterList,
            SERIALAPP_MAX_CLUSTERS, (cId_t *)SerialApp_ClusterList,
            FALSE );
    }
    
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    
    if ( keys & HAL_KEY_SW_4 )
    {
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
        
        // Initiate a Match Description Request (Service Discovery)
        txAddr.addrMode = AddrBroadcast;
        txAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
        ZDP_MatchDescReq( &txAddr, NWK_BROADCAST_SHORTADDR,
            SERIALAPP_PROFID,
            SERIALAPP_MAX_CLUSTERS, (cId_t *)SerialApp_ClusterList,
            SERIALAPP_MAX_CLUSTERS, (cId_t *)SerialApp_ClusterList,
            FALSE );
    }

}

void SerialApp_ProcessMSGCmd( afIncomingMSGPacket_t *pkt )
{
    uint8 delay;
    
    uint8 Data[10];
    uint8 afRxData[30]={0};
        Data[0] = 0xA5;
        Data[1] = 0x07;
        Data[5] = 0x0D;
        Data[6] = 0x0A;
    	
        switch ( pkt->clusterId )
	{
	case SERIALAPP_CLUSTERID:
        osal_memcpy(afRxData, pkt->cmd.Data, pkt->cmd.DataLength);
		switch(afRxData[0]) //简单协议命令字解析
		{
#if defined(ZDO_COORDINATOR)
		
		case 0xA5:     // 获取终端传感器数据
            if(afRxData[2] == 0x0A) //收到终端传过来的传感器数据并保存
            {  
                /*NodeData[0] = afRxData[3];
                NodeData[1] = afRxData[4];22222222222222222222222222222222222222222222222222222
                NodeData[2] = afRxData[5];
                NodeData[3] = afRxData[6];
                NodeData[4] = 0x00;*/
                if(afRxData[3] == 0x01)
                 {
                  NodeData1[0] = afRxData[3];
                  NodeData1[1] = afRxData[6];
                  NodeData1[2] = afRxData[7];
                  NodeData1[3] = afRxData[8];
                  NodeData1[4] = afRxData[9];
                  NodeData1[5] = afRxData[10];
                  NodeData1[6] = afRxData[13];
                  NodeData1[7] = afRxData[14];
                  NodeData1[8] = 0x00;
                 }
                
                if(afRxData[3] == 0x02)
                {
                NodeData2[0] = afRxData[3];
                NodeData2[1] = afRxData[11];
                NodeData2[2] = afRxData[12];
                NodeData2[3] = afRxData[4];
                NodeData2[4] = 0x00;
                }
                
                if(afRxData[3] == 0x03)
                {
                NodeData3[0] = afRxData[3];
                NodeData3[1] = afRxData[15];
                NodeData3[2] = afRxData[5];
                NodeData3[3] = afRxData[17];
                NodeData3[4] = afRxData[16];
                NodeData3[5] = 0x00;
                }
                
            /*    
               if( NodeData1[0]== 0x01 && NodeData1[3]<=0x01)
            {
              Data[2] = 0x01;
              Data[3] = 0x14;
              Data[4] = 0x01;
              if(lux_warn_flag)
                HalUARTWrite(UART0, Data, 7);
              lux_warn_flag=0;
             }
             // 光照低报警
             
             if( NodeData1[0]== 0x01 && NodeData1[3]==0x01&&!(lux_warn_flag))
            {
              Data[2] = 0x01;
              Data[3] = 0x14;
              Data[4] = 0x02;
              HalUARTWrite(UART0, Data, 7);
              lux_warn_flag=1;
               break;
            }
            //光照低报警解除      
            
            if(NodeData2[0]== 0x02 && NodeData2[3] == 1)
            {
               Data[2] = 0x02;
               Data[3] = 0x10;
               Data[4] = 0x01;
               if(pump_flag)
                HalUARTWrite(UART0, Data, 7);
               pump_flag=0;
               break;
            }
            if(NodeData1[0]== 0x01 && NodeData1[1] == 0&&!(pump_flag))
            {
                pump_flag=1;
            
            }
            //土壤湿度过低报警及解除                     
                
                        
             if(NodeData1[0]== 0x01 && NodeData1[6]>=0x1e)
             {
               Data[2] = 0x01;
               Data[3] = 0x12;
               Data[4] = 0x01;
               if(high_flag)
                HalUARTWrite(UART0, Data, 7);
               high_flag=0;//flag==0 the warnning has been taken place!
             }
             if(NodeData1[0]== 0x01 && NodeData1[6]==0x1d&&!(high_flag))
             {
               Data[2] = 0x01;
               Data[3] = 0x12;
               Data[4] = 0x02;
               HalUARTWrite(UART0, Data, 7);
               high_flag=1;
               break;
              } 
              //温度高报警及解除
                  
          if(NodeData1[0]== 0x01 && NodeData1[6]<0x14 && NodeData1[6]!= 0x00)
             {
               Data[2] = 0x01;
               Data[3] = 0x13;
               Data[4] = 0x01;
               if(low_flag)
                  HalUARTWrite(UART0, Data, 7);
               low_flag=0;
              }
              
            if(NodeData1[0]== 0x01 && NodeData1[6]==0x15&&!(low_flag))
             {
               Data[2] = 0x01;
               Data[3] = 0x13;
               Data[4] = 0x02;            
               HalUARTWrite(UART0, Data, 7);
               low_flag=1;
               break;
             }
             //温度低报警及解除
              
           if(NodeData1[0]== 0x01 && NodeData1[7]>=0x55)
             {
               Data[2] = 0x01;
               Data[3] = 0x15;
               Data[4] = 0x01;
               if(shihigh_flag)
                HalUARTWrite(UART0, Data, 7);
               shihigh_flag=0;//flag==0 the warnning has been taken place!
             }
             if(NodeData1[0]== 0x01 && NodeData1[7]==0x54&&!(shihigh_flag))
             {
               Data[2] = 0x01;
               Data[3] = 0x15;
               Data[4] = 0x02;
               HalUARTWrite(UART0, Data, 7);
               shihigh_flag=1;
               break;
              } 
              //湿度高报警及解除
                  
          if(NodeData1[0]== 0x01 && NodeData1[7]<0x1e && NodeData1[7]!= 0x00)
             {
               Data[2] = 0x01;
               Data[3] = 0x16;
               Data[4] = 0x01;
               if(shilow_flag)
                  HalUARTWrite(UART0, Data, 7);
               shilow_flag=0;
              }
              
            if(NodeData1[0]== 0x01 && NodeData1[7]==0x16&&!(shilow_flag))
             {
               Data[2] = 0x01;
               Data[3] = 0x16;
               Data[4] = 0x02;            
               HalUARTWrite(UART0, Data, 7);
               shilow_flag=1;
               break;
             }
             //湿度低报警及解除
           */   
              
            }
        #if UART_DEBUG
            HalUARTWrite (UART0, NodeData, 4); //调试时通过串口输出
            HalUARTWrite (UART0, "\n", 1);
        #endif            
           break;
#else  
		case 0xA5:  //开关灯设备          
        if(afRxData[3] == 0x30 || afRxData[3] == 0x31 || afRxData[3] == 0x32 || afRxData[3] == 0x33 || afRxData[3] == 0x34 || afRxData[3] == 0x35) //控制终端          
        {  
		if(afRxData[3] == 0x30 && afRxData[2] == 0x02 && EndDeviceID == 0x02)         
        {  
				if(afRxData[4] == 0x01)
                {
                     LED_PIN = 0;
				}
				if(afRxData[4] == 0x02)
                {
                     LED_PIN = 1;
				}			
        }		
        
         if(afRxData[3] == 0x31 && afRxData[2] == 0x03 && EndDeviceID == 0x03)         
        {  
				if(afRxData[4] == 0x01)
                {
                    heat_PIN = 0;
					}
				if(afRxData[4] == 0x02)
                {
                   heat_PIN = 1;
				}
        }	
        
         if(afRxData[3] == 0x32  && afRxData[2] == 0x02 && EndDeviceID == 0x02)         
        {  
				if(afRxData[4] == 0x01)
                {
                    Bfan_PIN = 1;
				}
				if(afRxData[4] == 0x02)
                {
                    Bfan_PIN = 0;
				}
        }	
        
         if(afRxData[3] == 0x33 && afRxData[2] == 0x03 && EndDeviceID == 0x03)         
        {  
				if(afRxData[4] == 0x01)
                {
                    Sfan_PIN = 0;
				}
				if(afRxData[4] == 0x02)
                {
                    Sfan_PIN = 1;
				}
		}	
        
         if(afRxData[3] == 0x34 && afRxData[2] == 0x03 && EndDeviceID == 0x03)         
        {  
				if(afRxData[4] == 0x01)
                {
                    DRIP_PIN = 0;
					}
				if(afRxData[4] == 0x02)
                {
                    DRIP_PIN = 1;
				}
		}	
        
         if(afRxData[3] == 0x35 && afRxData[2] == 0x03 && EndDeviceID == 0x03)         
        {  
				if(afRxData[4] == 0x01)
                {
                     DAMP_PIN = 0;
				}
				if(afRxData[4] == 0x02)
                {
                    DAMP_PIN = 1;
				}
		}	
			
			break;
        }		
#endif
        default :
            break;
        }
        break;
		// A response to a received serial data block.
		case SERIALAPP_CLUSTERID2:
			if ((pkt->cmd.Data[1] == SerialApp_TxSeq) &&
				((pkt->cmd.Data[0] == OTA_SUCCESS) || (pkt->cmd.Data[0] == OTA_DUP_MSG)))
			{
				SerialApp_TxLen = 0;
				osal_stop_timerEx(SerialApp_TaskID, SERIALAPP_SEND_EVT);
			}
			else
			{
				// Re-start timeout according to delay sent from other device.
				delay = BUILD_UINT16( pkt->cmd.Data[2], pkt->cmd.Data[3] );
				osal_start_timerEx( SerialApp_TaskID, SERIALAPP_SEND_EVT, delay );
			}
			break;
			
		default:
			break;
	}
}

uint8 TxBuffer[128];
uint8 Data[10];
uint8 SendData(uint8 AD,uint8 TP)
{
	//uint8 ret; //, i, index=4;

	TxBuffer[0] = 0xA5;
	TxBuffer[1] = 0x07;
	//TxBuffer[2] = addr;
	//TxBuffer[2] = TP;
        
        Data[0] = 0xA5;
        Data[1] = 0x07;
        Data[5] = 0x0D;
        Data[6] = 0x0A;
        

	/*switch(TP)
	{
	
        case 0x20:
         
          TxBuffer[2] = 0x01;
          TxBuffer[3] = NodeData[1];
          TxBuffer[4] = 0x0D;
          TxBuffer[5] = 0x0A;
          HalUARTWrite(UART0, TxBuffer, 6);
          ret = 1;
		break;
        case 0x80:
          TxBuffer[2] = 0x01;
          TxBuffer[3] = NodeData[0];
          TxBuffer[4] = 0x0D;
          TxBuffer[5] = 0x0A;
          
          HalUARTWrite(UART0, TxBuffer, 6);
          ret = 1;
          
          
	default:
        ret = 0;
		break;
	}*/
        if(AD == 0X01 && TP == 0x22)
        {
            TxBuffer[1] = 0x0B;                
            TxBuffer[2] = NodeData1[0];
            TxBuffer[3] = 0x03;
            TxBuffer[4] = NodeData1[1];
            TxBuffer[5] = NodeData1[2];
            TxBuffer[6] = NodeData1[3];
            TxBuffer[7] = NodeData1[4];
            TxBuffer[8] = NodeData1[5];
            TxBuffer[9] = 0x0D;
            TxBuffer[10] = 0x0A;
                
            HalUARTWrite(UART0, TxBuffer, 11);
        }
        
        if(AD == 0X01 && TP == 0x20)
          {
             
          
                TxBuffer[2] = NodeData1[0];
                TxBuffer[3] = 0x01;
                TxBuffer[4] = NodeData1[6];
                TxBuffer[5] = 0x0D;
                TxBuffer[6] = 0x0A;
                HalUARTWrite(UART0, TxBuffer, 7);
                //ret = 1;
                     
          }
        
        if(AD == 0X01 && TP == 0x21)
           {
                TxBuffer[2] = NodeData1[0];
                TxBuffer[3] = 0x02;
                TxBuffer[4] = NodeData1[7];
                TxBuffer[5] = 0x0D;
                TxBuffer[6] = 0x0A;
                HalUARTWrite(UART0, TxBuffer, 7);
                //ret = 1;
            }
        

        
        

    return 0;
}

/*********************************************************************
* @fn      SerialApp_Send
*
* @brief   Send data OTA.
*
* @param   none
*
* @return  none
*/
static void SerialApp_Send(void)
{
    uint8  TP;
    uint8  AD;
    uint8 data[10];
    data[0] = 0xA5;
    data[1] = 0x07;
    data[5] = 0x0D;
    data[6] = 0x0A;
    
    //uint8 checksum=0;
	
#if SERIAL_APP_LOOPBACK
	if (SerialApp_TxLen < SERIAL_APP_TX_MAX)
	{
		SerialApp_TxLen += HalUARTRead(SERIAL_APP_PORT, SerialApp_TxBuf+SerialApp_TxLen+1,
			SERIAL_APP_TX_MAX-SerialApp_TxLen);
	}
	
	if (SerialApp_TxLen)
	{
		(void)SerialApp_TxAddr;
		if (HalUARTWrite(SERIAL_APP_PORT, SerialApp_TxBuf+1, SerialApp_TxLen))
		{
			SerialApp_TxLen = 0;
		}
		else
		{
			osal_set_event(SerialApp_TaskID, SERIALAPP_SEND_EVT);
		}
	}
#else
	if (!SerialApp_TxLen && 
		(SerialApp_TxLen = HalUARTRead(UART0, SerialApp_TxBuf, SERIAL_APP_TX_MAX)))
	{
        if (SerialApp_TxLen)
        {
            SerialApp_TxLen = 0;
            if(SerialApp_TxBuf[0] == 0xA5)
            {
				//addr = SerialApp_TxBuf[2];
		      AD = SerialApp_TxBuf[2];	
                      TP = SerialApp_TxBuf[3];
                //len = GetDataLen(TP); 
                //len += 4;
                //checksum = XorCheckSum(SerialApp_TxBuf, len);
                
				//接收数据正确返回相应数据
                //if(checksum == SerialApp_TxBuf[len] && SerialApp_TxBuf[len+1] == 0x23)
                //{
                    if(TP == 0x30 || TP == 0x31 || TP == 0x32 || TP == 0x33 || TP == 0x34 || TP == 0x35) //控制终端
                    {                            
                        if (afStatus_SUCCESS == AF_DataRequest(&Broadcast_DstAddr,
                                                (endPointDesc_t *)&SerialApp_epDesc,
                                                SERIALAPP_CLUSTERID,
                                                7, 
                                                SerialApp_TxBuf,
                                                &SerialApp_MsgID,
                                                0,
                                                AF_DEFAULT_RADIUS))
                        {
                           if(TP == 0x30) //如果开启自动刷新则不需要这步操作
                                //NodeData[3] = SerialApp_TxBuf[3];  //更新缓冲区灯的状态
                            {   data[2] = 0x02;
                                data[3] = 0x50;
                                data[4] = SerialApp_TxBuf[4];
                            MicroWait (5000); 

                            HalUARTWrite(UART0, data, 7); //无线发送成功后原样返回给上位机	
                            //osal_set_event(SerialApp_TaskID, SERIALAPP_SEND_EVT);
                            }
                            
                            if(TP == 0x31) //如果开启自动刷新则不需要这步操作
                                //NodeData[3] = SerialApp_TxBuf[3];  //更新缓冲区灯的状态
                            {   data[2] = 0x03;
                                data[3] = 0x51;
                                data[4] = SerialApp_TxBuf[4];
                             MicroWait (5000); 
 
                            HalUARTWrite(UART0, data, 7); //无线发送成功后原样返回给上位机	
                            //osal_set_event(SerialApp_TaskID, SERIALAPP_SEND_EVT);
                            }
                            
                            if(TP == 0x32) //如果开启自动刷新则不需要这步操作
                                //NodeData[3] = SerialApp_TxBuf[3];  //更新缓冲区灯的状态
                            {data[2] = 0x02;
                                data[3] = 0x52;
                                data[4] = SerialApp_TxBuf[4];
                             MicroWait (5000); 
 
                            HalUARTWrite(UART0, data, 7); //无线发送成功后原样返回给上位机	
                            //osal_set_event(SerialApp_TaskID, SERIALAPP_SEND_EVT);
                            }
                            
                            if(TP == 0x33) //如果开启自动刷新则不需要这步操作
                                //NodeData[3] = SerialApp_TxBuf[3];  //更新缓冲区灯的状态
                            {data[2] = 0x03;
                                data[3] = 0x53;
                                data[4] = SerialApp_TxBuf[4];
                              MicroWait (5000); 

                            HalUARTWrite(UART0, data, 7); //无线发送成功后原样返回给上位机	
                            //osal_set_event(SerialApp_TaskID, SERIALAPP_SEND_EVT);
                            }
                            
                            if(TP == 0x34) //如果开启自动刷新则不需要这步操作
                                //NodeData[3] = SerialApp_TxBuf[3];  //更新缓冲区灯的状态
                            {data[2] = 0x03;
                                data[3] = 0x54;
                                data[4] = SerialApp_TxBuf[4];
                              MicroWait (5000); 

                            HalUARTWrite(UART0, data, 7); //无线发送成功后原样返回给上位机	
                            //osal_set_event(SerialApp_TaskID, SERIALAPP_SEND_EVT);
                            }
                            
                            if(TP == 0x35) //如果开启自动刷新则不需要这步操作
                                //NodeData[3] = SerialApp_TxBuf[3];  //更新缓冲区灯的状态
                            {data[2] = 0x03;
                                data[3] = 0x55;
                                data[4] = SerialApp_TxBuf[4];
                              MicroWait (5000); 

                           HalUARTWrite(UART0, data, 7); //无线发送成功后原样返回给上位机	
                            //osal_set_event(SerialApp_TaskID, SERIALAPP_SEND_EVT);
                            }  
                        }
                        else  //暂时没发现错误，关闭终端发送也正常。无线发送失败后将数据位和校验位置0返给上位机	
                        {
                            /*SerialApp_TxBuf[len-1] = 0x00;
                            SerialApp_TxBuf[len] = 0x00;
                            HalUARTWrite(UART0, SerialApp_TxBuf, len+2);*/
                        }
                    }
                    else
                    {
					    SendData(AD,TP);   //查询操作
                    }
				//}
			}
		}
    }
#endif
}

/*********************************************************************
* @fn      SerialApp_Resp
*
* @brief   Send data OTA.
*
* @param   none
*
* @return  none
*/
static void SerialApp_Resp(void)
{
	if (afStatus_SUCCESS != AF_DataRequest(&SerialApp_RxAddr,
		(endPointDesc_t *)&SerialApp_epDesc,
		SERIALAPP_CLUSTERID2,
		SERIAL_APP_RSP_CNT, SerialApp_RspBuf,
		&SerialApp_MsgID, 0, AF_DEFAULT_RADIUS))
	{
		osal_set_event(SerialApp_TaskID, SERIALAPP_RESP_EVT);
	}
}

/*********************************************************************
* @fn      SerialApp_CallBack
*
* @brief   Send data OTA.
*
* @param   port - UART port.
* @param   event - the UART port event flag.
*
* @return  none
*/
static void SerialApp_CallBack(uint8 port, uint8 event)
{
	(void)port;
	
	if ((event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT)) &&
#if SERIAL_APP_LOOPBACK
		(SerialApp_TxLen < SERIAL_APP_TX_MAX))
#else
		!SerialApp_TxLen)
#endif
	{
		SerialApp_Send();
	}
}


//------------------------------------------------------------------------------------------------------------------------------------------
//查询单个终端上所有传感器的数据 3A 00 01 02 XX 23  响应：3A 00 01 02 00 00 00 00 xor 23
void SerialApp_SendPeriodicMessage( void )
{
    uint8 SendBuf[20]={0};
    unsigned long int sun=0;
    unsigned char lux[5];
    char strTemp[16]="Temperature: ";
    char strHumidity[13]="Humidity: ";
    char strLUX[16]="Lux: ";
    char strLEDon[10]="LED: ON";
    char strLEDoff[10]="LED: OFF";
    char SD_shi,SD_ge,WD_shi,WD_ge=4;
    //unsigned long int sunbuf[1];
    SendBuf[0] = 0xA5;                          
    SendBuf[1] = 0x07;
    SendBuf[2] = 0x0A;
    DHT11(); 
    SendBuf[3] = EndDeviceID;  
    SendBuf[4] = GetHUM();  
    SendBuf[5] = GetDRIP();
    
    
    void B_LUX_Init();
    sun=B_LUX_GetLux();
    
    lux[0]=(sun/10000);
    lux[1]=((sun%10000)/1000);
    lux[2]=(((sun%10000)%1000)/100);
    lux[3]=((((sun%10000)%1000)%100)/10);
    lux[4]=((((sun%10000)%1000)%100)%10);
     
    SendBuf[6] = lux[0];    
    SendBuf[7] = lux[1]; 
    SendBuf[8] = lux[2];
    SendBuf[9] = lux[3];
    SendBuf[10] = lux[4];
    SendBuf[11] = GetLED();  
    SendBuf[12] = GetBfan();  
    SendBuf[13] = wendu;   
    SendBuf[14] = shidu;
    SendBuf[15] = GetSfan();  
    SendBuf[16] = Getheat();  
    SendBuf[17] = GetDAMP();

    //温湿度值转换十进制 并显示
    WD_shi=wendu/10; 
    WD_ge=wendu%10;        
    SD_shi=shidu/10; 
    SD_ge=shidu%10;     
    strTemp[13]=WD_shi+0x30;
    strTemp[14]=WD_ge+0x30;
    strHumidity[10]=SD_shi+0x30;
    strHumidity[11]=SD_ge+0x30;
    
    HalLcdWriteString(strTemp,HAL_LCD_LINE_3);//温度显示
    HalLcdWriteString(strHumidity,HAL_LCD_LINE_4);//湿度显示
    
    
    //光照强度显示
    strLUX[5]=(sun/10000)+0x30;
    strLUX[6]=((sun%10000)/1000)+0x30;
    strLUX[7]=(((sun%10000)%1000)/100)+0x30;
    strLUX[8]=((((sun%10000)%1000)%100)/10)+0x30;
    strLUX[9]=((((sun%10000)%1000)%100)%10)+0x30;
       
    //HalLcdWriteString(strLUX,HAL_LCD_LINE_3);//光照强度显示
    
   // HalUARTWrite(0, (uint8 *)lux,5);//串口输出光照强度值
    
    
    
    //led工作状态 显示
    /*
    if (SendBuf[11]==0)
    {
      HalLcdWriteString(strLEDon,HAL_LCD_LINE_3); 
    }
    else
    {
      HalLcdWriteString(strLEDoff,HAL_LCD_LINE_3);
    }
    */
        
    
    SerialApp_TxAddr.addrMode = (afAddrMode_t)Addr16Bit;
    SerialApp_TxAddr.endPoint = SERIALAPP_ENDPOINT;
    SerialApp_TxAddr.addr.shortAddr = 0x00;  
    if ( AF_DataRequest( &SerialApp_TxAddr, (endPointDesc_t *)&SerialApp_epDesc,
               SERIALAPP_CLUSTERID,
               20,
               SendBuf,
               &SerialApp_MsgID, 
               0, 
               AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    // Successfully requested to be sent.
    }
    else
    {
    // Error occurred in request to send.
    }
}



//通过串口输出短地址 IEEE
/*void PrintAddrInfo(uint16 shortAddr, uint8 *pIeeeAddr)
{
    uint8 strIeeeAddr[17] = {0};
    char  buff[30] = {0};    
    
    //获得短地址   
    sprintf(buff, "shortAddr:%04X   IEEE:", shortAddr);  
 
    //获得IEEE地址
    GetIeeeAddr(pIeeeAddr, strIeeeAddr);

    HalUARTWrite (UART0, (uint8 *)buff, strlen(buff));
    Delay_ms(10);
    HalUARTWrite (UART0, strIeeeAddr, 16); 
    HalUARTWrite (UART0, "\n", 1);
}
*/
/*void AfSendAddrInfo(void)
{
    uint16 shortAddr;
    uint8 strBuf[11]={0};  
    
    SerialApp_TxAddr.addrMode = (afAddrMode_t)Addr16Bit;
    SerialApp_TxAddr.endPoint = SERIALAPP_ENDPOINT;
    SerialApp_TxAddr.addr.shortAddr = 0x00;   
    
    shortAddr=NLME_GetShortAddr();
    
    strBuf[0] = 0x3B;                          //发送地址给协调器 可用于点播
    strBuf[1] = HI_UINT16( shortAddr );        //存放短地址高8位
    strBuf[2] = LO_UINT16( shortAddr );        //存放短地址低8位
    
    osal_memcpy(&strBuf[3], NLME_GetExtAddr(), 8);
        
   if ( AF_DataRequest( &SerialApp_TxAddr, (endPointDesc_t *)&SerialApp_epDesc,
                       SERIALAPP_CLUSTERID,
                       11,
                       strBuf,
                       &SerialApp_MsgID, 
                       0, 
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }   
}
*/
/*void GetIeeeAddr(uint8 * pIeeeAddr, uint8 *pStr)
{
  uint8 i;
  uint8 *xad = pIeeeAddr;

  for (i = 0; i < Z_EXTADDR_LEN*2; xad--)
  {
    uint8 ch;
    ch = (*xad >> 4) & 0x0F;
    *pStr++ = ch + (( ch < 10 ) ? '0' : '7');
    i++;
    ch = *xad & 0x0F;
    *pStr++ = ch + (( ch < 10 ) ? '0' : '7');
    i++;
  }
}
*/
/*uint8 XorCheckSum(uint8 * pBuf, uint8 len)
{
	uint8 i;
	uint8 byRet=0;

	if(len == 0)
		return byRet;
	else
		byRet = pBuf[0];

	for(i = 1; i < len; i ++)
		byRet = byRet ^ pBuf[i];

	return byRet;
}
*/
/*uint8 GetDataLen(uint8 fc)
{
    uint8 len=0;
    switch(fc)
    {
    case 0x0A:
    case 0x0B:
    case 0x0C:
    case 0x0D:
      len = 1;
      break;
    }
    
    return len;
}
*/

//获得P0_4 继电器引脚的电平
/*uint8 GetLamp( void )
{
  uint8 ret;
  
  if(LAMP_PIN == 0)
    ret = 0;
  else
    ret = 1;
  
  return ret;
}

//获得P0_5 MQ-2气体传感器的数据
uint8 GetGas( void )
{
  uint8 ret;
  
  if(GAS_PIN == 0)
    ret = 0;
  else
    ret = 1;
  
  return ret;
}*/

uint8 GetHUM( void )
{
  uint8 ret;
  
  if(HUM_PIN == 0)
    ret = 0;
  else
    ret = 1;
  
  return ret;
}

uint8 GetDRIP( void )
{
  uint8 ret;
  
  if(DRIP_PIN == 0)
    ret = 0;
  else
    ret = 1;
  
  return ret;
}

uint8 GetLED( void )
{
  uint8 ret;
  
  if(LED_PIN == 0)
    ret = 0;
  else
    ret = 1;
  
  return ret;
}

uint8 Getheat( void )
{
  uint8 ret;
  
  if(heat_PIN == 0)
    ret = 0;
  else
    ret = 1;
  
  return ret;
}

uint8 GetDAMP( void )
{
  uint8 ret;
  
  if(DAMP_PIN == 0)
    ret = 0;
  else
    ret = 1;
  
  return ret;
}


uint8 GetBfan( void )
{
  uint8 ret;
  
  if(Bfan_PIN == 0)
    ret = 0;
  else
    ret = 1;
  
  return ret;
}

uint8 GetSfan( void )
{
  uint8 ret;
  
  if(Sfan_PIN == 0)
    ret = 0;
  else
    ret = 1;
  
  return ret;
}
//-------------------------------------------------------------------



/*********************************************************************
*********************************************************************/
