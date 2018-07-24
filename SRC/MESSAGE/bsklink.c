/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     bsklink.c
 * @说明     bsklink飞控数据传输协议
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.06
**********************************************************************************************************/
#include "bsklink.h"
#include <string.h>

/**********************************************************************************************************
*函 数 名: BsklinkMsgCalculateSum
*功能说明: 计算校验和
*形    参: 消息结构体指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkMsgCalculateSum(BSKLINK_MSG_t* msg)
{
    uint8_t length = msg->length + 6;
    uint8_t temp[BSKLINK_MAX_PAYLOAD_LENGTH+10];
    
    memcpy(temp, msg, length);
    
    msg->checksum = 0;
    
    for(uint8_t i=0;i<length;i++)
    {
        msg->checksum += temp[i];
    }
}

/**********************************************************************************************************
*函 数 名: BsklinkMsgCheckSum
*功能说明: 和校验
*形    参: 消息结构体指针
*返 回 值: 校验结果
**********************************************************************************************************/
bool BsklinkMsgCheckSum(BSKLINK_MSG_t* msg)
{
    uint8_t length = msg->length + 6;
    uint8_t temp[BSKLINK_MAX_PAYLOAD_LENGTH+10];
    uint8_t checksum = 0;
    
    memcpy(temp, msg, length);
    
    for(uint8_t i=0;i<length;i++)
    {
        checksum += temp[i];
    }
    
    if(checksum == msg->checksum)
        return true;
    else
        return false;
}

/**********************************************************************************************************
*函 数 名: BsklinkMsgFormat
*功能说明: 帧格式化输出
*形    参: 消息结构体 缓存数组指针
*返 回 值: 无
**********************************************************************************************************/
void BsklinkMsgFormat(BSKLINK_MSG_t msg, uint8_t* msgTemp)
{ 
    //首字节为帧长度
    msgTemp[0] = msg.length + 7;
    
    msgTemp[1] = msg.head1;
    msgTemp[2] = msg.head2;
    msgTemp[3] = msg.deviceid;
	msgTemp[4] = msg.sysid;
    msgTemp[5] = msg.msgid;
    msgTemp[6] = msg.length;
    
    memcpy(msgTemp+7, msg.payload, msg.length);
    msgTemp[7+msg.length] = msg.checksum;
}

/**********************************************************************************************************
*函 数 名: BsklinkParseChar
*功能说明: 单字节解析
*形    参: 消息结构体指针 输入数据
*返 回 值: 标志位 一帧接收完成返回true
**********************************************************************************************************/
bool BsklinkParseChar(BSKLINK_MSG_t* msg, uint8_t data)
{     
    switch(msg->recvStatus)
    {
        case 0:     //帧头
            if(data == BSKLINK_MSG_HEAD_1)
            {
                msg->head1 = data;
                msg->recvStatus++;
            }
            break;
        case 1:     //帧头
            if(data == BSKLINK_MSG_HEAD_2)
            {
                msg->head2 = data;
                msg->recvStatus++;
            } 
            else
                msg->recvStatus = 0;
            break;
        case 2:     //设备ID
            msg->deviceid = data;
            msg->recvStatus++;
            break;
		case 3:     //系统ID
            msg->sysid = data;
            msg->recvStatus++;
            break;
        case 4:     //消息ID
            msg->msgid = data;
            msg->recvStatus++;
            break;
        case 5:     //数据负载长度
            msg->length = data;
            msg->recvStatus++;
            break;
        case 6:     //数据负载接收
            msg->payload[msg->payloadRecvCnt++] = data;
            if(msg->payloadRecvCnt == msg->length)
            {
                msg->payloadRecvCnt = 0;
                msg->recvStatus++;
            }
            break;
        case 7:     //帧校验
            msg->checksum = data;
            msg->recvStatus = 0;
            if(BsklinkMsgCheckSum(msg))
            {
                return true;
            } 
        default:
            msg->recvStatus = 0;
            msg->payloadRecvCnt = 0;
            break;        
    }
    
    return false;
}

