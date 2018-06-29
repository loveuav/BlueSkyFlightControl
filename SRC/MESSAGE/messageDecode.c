/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     messageDecoded.c
 * @说明     接收数据解析
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.06
**********************************************************************************************************/
#include "messageDecode.h"
#include "bsklink.h"

/**********************************************************************************************************
*函 数 名: MessageDecode
*功能说明: 消息解析
*形    参: 接收数据
*返 回 值: 无
**********************************************************************************************************/
void MessageDecode(uint8_t data)
{
    static BSKLINK_MSG_t msg;
    static uint32_t i=0;
    
    if(BsklinkDecode(&msg, data))
    {
        if(msg.msgid == BSKLINK_MSG_ID_FLIGHT_DATA)
        {
            i++;
        }
        
        if(msg.msgid == BSKLINK_MSG_ID_RC_DATA)
        {
            i++;
        }
    }
}


