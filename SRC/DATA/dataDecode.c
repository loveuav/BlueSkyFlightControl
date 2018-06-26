/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     dataDecoded.c
 * @说明     接收数据解析
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.06
**********************************************************************************************************/
#include "dataDecode.h"
#include "bsklink.h"

void dataDecode(uint8_t data)
{
    static BSKLINK_MSG_t msg;
    static uint8_t i=0;
    
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


