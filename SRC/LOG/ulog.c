/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     ulog.c
 * @说明     ulog日志
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.08
**********************************************************************************************************/
#include "ulog.h"
#include "logger.h"
#include "board.h"

/**********************************************************************************************************
*函 数 名: UlogWriteHeader
*功能说明: ulog写入日志固定头部信息
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void UlogWriteHeader(void)
{
	ulog_file_header_s header;
    
	header.magic[0] = 'U';
	header.magic[1] = 'L';
	header.magic[2] = 'o';
	header.magic[3] = 'g';
	header.magic[4] = 0x01;
	header.magic[5] = 0x12;
	header.magic[6] = 0x35;
	header.magic[7] = 0x01; //版本1
	header.timestamp = GetSysTimeUs();

    LoggerWrite(&header, sizeof(header));
	//write_message(&header, sizeof(header));

}



