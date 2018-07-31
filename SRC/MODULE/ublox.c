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
            
            default:
                break;
        }
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

UBLOX_t Ublox_GetData(void)
{
	return ublox;
}

