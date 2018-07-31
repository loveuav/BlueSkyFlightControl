/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     ublox.c
 * @说明     GPS数据协议解析，目前只支持UBLOX协议，后续增加对NMEA协议的支持        
 * @版本  	 V1.1
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.07 
**********************************************************************************************************/
#include "ublox.h"
#include "drv_usart.h"

//UBLOX协议参考：lindi.iki.fi/lindi/gps/ubx.html

UBLOX_t ublox;

static void Ublox_Decode(uint8_t data);
static void UbloxEnableMessage(uint8_t class, uint8_t id, uint8_t rate);
static void UbloxSetRate(uint16_t rate);
static void UbloxSetPrt(uint32_t baudrate);

/**********************************************************************************************************
*函 数 名: Ublox_Init
*功能说明: ublox数据解析初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Ublox_Init(void)
{
    //设置GPS串口接收中断回调函数（即数据协议解析函数）
    Usart_SetIRQCallback(GPS_UART, Ublox_Decode);
    
    OsDelayMs(20);
    
    //使能ublox消息输出
    UbloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_NAV_POSLLH, 1);
    OsDelayMs(20);
    UbloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_NAV_VALNED, 1);
    OsDelayMs(20);
    UbloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_NAV_SOL, 1);
    OsDelayMs(20);
    UbloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_NAV_DOP, 0);
    OsDelayMs(20);

    //设置ublox串口波特率   
    UbloxSetPrt(115200);
    OsDelayMs(20);
    
    //设置ublox输出速率：ms
    UbloxSetRate(100);
}

/**********************************************************************************************************
*函 数 名: Ublox_PayloadDecode
*功能说明: ublox数据负载解析
*形    参: 输入数据
*返 回 值: 无
**********************************************************************************************************/
static void Ublox_PayloadDecode(UBLOX_t_RAW_t ubloxRawData)
{
    if(ubloxRawData.class == UBLOX_NAV_CLASS)
    {
        switch(ubloxRawData.id)
        {
            case UBLOX_NAV_POSLLH:
                ublox.time      = (float)ubloxRawData.payload.posllh.iTOW / 1000;
                ublox.longitude = (double)ubloxRawData.payload.posllh.lat * (double)1e-7;
                ublox.latitude  = (double)ubloxRawData.payload.posllh.lon * (double)1e-7;
                ublox.altitude  = (float)ubloxRawData.payload.posllh.hMSL * 0.001f;
                ublox.hAcc      = (float)ubloxRawData.payload.posllh.hAcc * 0.001f;
                ublox.vAcc      = (float)ubloxRawData.payload.posllh.vAcc * 0.001f;
                break;
            
            case UBLOX_NAV_VALNED:
                ublox.velN    = ubloxRawData.payload.velned.velN;          
                ublox.velE    = ubloxRawData.payload.velned.velE;         
                ublox.velD    = ubloxRawData.payload.velned.velD;     
                ublox.speed   = ubloxRawData.payload.velned.gSpeed;   
                ublox.heading = ubloxRawData.payload.velned.heading * 1e-5f;
                ublox.sAcc    = ubloxRawData.payload.velned.sAcc * 0.01f;     
                ublox.cAcc    = ubloxRawData.payload.velned.cAcc * 1e-5f;
                break;

            case UBLOX_NAV_SOL:
                ublox.numSV     = ubloxRawData.payload.sol.numSV;
                ublox.fixStatus = ubloxRawData.payload.sol.gpsFix;
                break;
            
            case UBLOX_NAV_DOP:
                break;
            
            default:
                break;
        }
    }
    else if(ubloxRawData.class == UBLOX_CFG_CLASS)
    {
    }
}

/**********************************************************************************************************
*函 数 名: Ublox_Decode
*功能说明: ublox协议解析
*形    参: 输入数据
*返 回 值: 无
**********************************************************************************************************/
static void Ublox_Decode(uint8_t data)
{
    static UBLOX_t_RAW_t ubloxRaw;
    
    switch (ubloxRaw.state) 
    {
        /*帧头1*/
        case UBLOX_WAIT_SYNC1:
            if (data == UBLOX_SYNC1)
                ubloxRaw.state = UBLOX_WAIT_SYNC2;
            break;
           
        /*帧头2*/
        case UBLOX_WAIT_SYNC2:
            if (data == UBLOX_SYNC2)
                ubloxRaw.state = UBLOX_WAIT_CLASS;
            else			
                ubloxRaw.state = UBLOX_WAIT_SYNC1;
            break;
        
        /*消息类型*/            
        case UBLOX_WAIT_CLASS:
            ubloxRaw.class = data;
            //校验值初始化
            ubloxRaw.ubloxRxCK_A = 0;
            ubloxRaw.ubloxRxCK_B = 0;
            //校验值计算
            ubloxRaw.ubloxRxCK_A += data;
            ubloxRaw.ubloxRxCK_B += ubloxRaw.ubloxRxCK_A;
            ubloxRaw.state = UBLOX_WAIT_ID;
            break;

        /*消息ID*/  
        case UBLOX_WAIT_ID:
            ubloxRaw.id = data;
            //校验值计算
            ubloxRaw.ubloxRxCK_A += data;
            ubloxRaw.ubloxRxCK_B += ubloxRaw.ubloxRxCK_A;
            ubloxRaw.state = UBLOX_WAIT_LEN1;
            break;

        /*消息长度低8位*/  
        case UBLOX_WAIT_LEN1:
            ubloxRaw.length = data;
            //校验值计算
            ubloxRaw.ubloxRxCK_A += data;
            ubloxRaw.ubloxRxCK_B += ubloxRaw.ubloxRxCK_A;
            ubloxRaw.state = UBLOX_WAIT_LEN2;
            break;

        /*消息长度高8位*/ 
        case UBLOX_WAIT_LEN2:
            ubloxRaw.length += (data << 8);
            //校验值计算
            ubloxRaw.ubloxRxCK_A += data;
            ubloxRaw.ubloxRxCK_B += ubloxRaw.ubloxRxCK_A;
            if (ubloxRaw.length >= (UBLOX_MAX_PAYLOAD-1))
            {
                ubloxRaw.length = 0;
                ubloxRaw.state = UBLOX_WAIT_SYNC1;
            } 
            else if (ubloxRaw.length > 0) 
            {
                ubloxRaw.count = 0;
                ubloxRaw.state = UBLOX_PAYLOAD;
            }
            else 
            {
                ubloxRaw.state = UBLOX_CHECK1;
            }
            break;

        /*消息负载*/
        case UBLOX_PAYLOAD:
            *((char *)(&ubloxRaw.payload) + ubloxRaw.count) = data;
            if (++ubloxRaw.count == ubloxRaw.length)
            {
                ubloxRaw.state = UBLOX_CHECK1;
            } 
            //校验值计算           
            ubloxRaw.ubloxRxCK_A += data;
            ubloxRaw.ubloxRxCK_B += ubloxRaw.ubloxRxCK_A;
            break;

        /*CKA校验位对比*/    
        case UBLOX_CHECK1:
            if (data == ubloxRaw.ubloxRxCK_A) 
            {
                ubloxRaw.state = UBLOX_CHECK2;
            }
            else 
            {
                ubloxRaw.state = UBLOX_WAIT_SYNC1;
                ubloxRaw.checksumErrors++;
            }
            break;
            
        /*CKB校验位对比*/
        case UBLOX_CHECK2:
            ubloxRaw.state = UBLOX_WAIT_SYNC1;
            if (data == ubloxRaw.ubloxRxCK_B) 
            {
                //接收完毕，解析数据负载
                Ublox_PayloadDecode(ubloxRaw);
            }
            else 
            {
                ubloxRaw.checksumErrors++;
            }
            break;

        default:  
            break;
    }
}

/**********************************************************************************************************
*函 数 名: Ublox_SendData
*功能说明: 发送数据给ublox模块
*形    参: 数据缓冲区指针 数据长度
*返 回 值: 无
**********************************************************************************************************/
void Ublox_SendData(uint8_t *data, uint8_t length)
{
    //串口发送
    Usart_SendData(GPS_UART, data, length);
}

/**********************************************************************************************************
*函 数 名: Ublox_GetData
*功能说明: 获取ublox数据
*形    参: 无
*返 回 值: gps数据
**********************************************************************************************************/
UBLOX_t Ublox_GetData(void)
{
	return ublox;
}

/**********************************************************************************************************
*函 数 名: UbloxEnableMessage
*功能说明: 使能ublox消息输出
*形    参: 消息类型 消息id 输出速率（0表示不输出，1表示一个周期输出一次（最快），数值越大输出速率越低）
*返 回 值: 无
**********************************************************************************************************/
static void UbloxEnableMessage(uint8_t class, uint8_t id, uint8_t rate) 
{
    uint8_t ubloxData[50];
    uint8_t dataCnt = 0;
    uint8_t ck_a = 0, ck_b = 0;
    
    ubloxData[dataCnt++] = UBLOX_SYNC1;     //帧头1
    ubloxData[dataCnt++] = UBLOX_SYNC2;     //帧头2
    ubloxData[dataCnt++] = UBLOX_CFG_CLASS; //消息类型
    ubloxData[dataCnt++] = UBLOX_CFG_MSG;   //消息id
    ubloxData[dataCnt++] = 0x03;            //消息负载长度低8位
    ubloxData[dataCnt++] = 0x00;            //消息负载长度高8位

    ubloxData[dataCnt++] = class;           //设置消息类型
    ubloxData[dataCnt++] = id;              //设置消息id
    ubloxData[dataCnt++] = rate;            //设置消息输出速率

    for(uint8_t i=2; i<dataCnt; i++)
    {
        ck_a += ubloxData[i];
        ck_b += ck_a;
    }
    
    ubloxData[dataCnt++] = ck_a; 
    ubloxData[dataCnt++] = ck_b; 

    Ublox_SendData(ubloxData, dataCnt);
}

/**********************************************************************************************************
*函 数 名: UbloxSetRate
*功能说明: 设置ublox输出速率
*形    参: 输出速率（单位：ms）
*返 回 值: 无
**********************************************************************************************************/
static void UbloxSetRate(uint16_t rate) 
{
    uint8_t ubloxData[50];
    uint8_t dataCnt = 0;
    uint8_t ck_a = 0, ck_b = 0;
    
    ubloxData[dataCnt++] = UBLOX_SYNC1;             //帧头1
    ubloxData[dataCnt++] = UBLOX_SYNC2;             //帧头2
    ubloxData[dataCnt++] = UBLOX_CFG_CLASS;         //消息类型
    ubloxData[dataCnt++] = UBLOX_CFG_RTATE;         //消息id
    ubloxData[dataCnt++] = 0x06;                    //消息负载长度低8位
    ubloxData[dataCnt++] = 0x00;                    //消息负载长度高8位

    ubloxData[dataCnt++] = (uint8_t)rate;           //输出速率低八位
    ubloxData[dataCnt++] = (uint8_t)(rate >> 8);    //输出速率高八位
    ubloxData[dataCnt++] = 0x01;                    //导航周期低八位
    ubloxData[dataCnt++] = 0x00;                    //导航周期高八位
    ubloxData[dataCnt++] = 0x01;                    //timeRef 0:UTC, 1:GPS time
    ubloxData[dataCnt++] = 0x00;                    //高八位
//    
    for(uint8_t i=2; i<dataCnt; i++)
    {
        ck_a += ubloxData[i];
        ck_b += ck_a;
    }
    
    ubloxData[dataCnt++] = ck_a; 
    ubloxData[dataCnt++] = ck_b; 

    Ublox_SendData(ubloxData, dataCnt);
}

/**********************************************************************************************************
*函 数 名: UbloxSetPrt
*功能说明: 设置ublox输出配置
*形    参: 波特率
*返 回 值: 无
**********************************************************************************************************/
static void UbloxSetPrt(uint32_t baudrate) 
{
    uint8_t ubloxData[50];
    uint8_t dataCnt = 0;
    uint8_t ck_a = 0, ck_b = 0;
    
    ubloxData[dataCnt++] = UBLOX_SYNC1;               //帧头1
    ubloxData[dataCnt++] = UBLOX_SYNC2;               //帧头2
    ubloxData[dataCnt++] = UBLOX_CFG_CLASS;           //消息类型
    ubloxData[dataCnt++] = UBLOX_CFG_PRT;             //消息id
    ubloxData[dataCnt++] = 0x14;                      //消息负载长度低8位
    ubloxData[dataCnt++] = 0x00;                      //消息负载长度高8位

    ubloxData[dataCnt++] = 0x01;                      //端口号
    ubloxData[dataCnt++] = 0x00;                      //预留
    ubloxData[dataCnt++] = 0x00;                      //TX Ready高八位
    ubloxData[dataCnt++] = 0x00;                      //TX Ready低八位
    ubloxData[dataCnt++] = (uint8_t)(0x08D0);         //串口配置
    ubloxData[dataCnt++] = (uint8_t)(0x08D0 >> 8);    //8位
    ubloxData[dataCnt++] = (uint8_t)(0x08D0 >> 16);   //1个停止位
    ubloxData[dataCnt++] = (uint8_t)(0x08D0 >> 24);   //无校验位
    ubloxData[dataCnt++] = (uint8_t)(baudrate);       //串口波特率
    ubloxData[dataCnt++] = (uint8_t)(baudrate >> 8);  //
    ubloxData[dataCnt++] = (uint8_t)(baudrate >> 16); //
    ubloxData[dataCnt++] = (uint8_t)(baudrate >> 24); //
    ubloxData[dataCnt++] = 0x07;                      //输入协议配置
    ubloxData[dataCnt++] = 0x00;                      //ubx+nmea+rtcm
    ubloxData[dataCnt++] = 0x01;                      //输出协议配置
    ubloxData[dataCnt++] = 0x00;                      //ubx
    ubloxData[dataCnt++] = 0x00;                      //flags低八位
    ubloxData[dataCnt++] = 0x00;                      //flags高八位
    ubloxData[dataCnt++] = 0x00;                      //reserved
    ubloxData[dataCnt++] = 0x00;                      //reserved
    
    for(uint8_t i=2; i<dataCnt; i++)
    {
        ck_a += ubloxData[i];
        ck_b += ck_a;
    }
    
    ubloxData[dataCnt++] = ck_a; 
    ubloxData[dataCnt++] = ck_b; 

    Ublox_SendData(ubloxData, dataCnt);
}

