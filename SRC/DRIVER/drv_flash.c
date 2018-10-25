/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     drv_flash.c
 * @说明     单片机Flash读写驱动
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05
**********************************************************************************************************/
#include "drv_flash.h"

/**********************************************************************************************************
*函 数 名: Flash_ReadByte
*功能说明: 从flash中读取一个字节
*形    参: 起始地址 地址偏移量
*返 回 值: 读取到的字节
**********************************************************************************************************/
uint8_t Flash_ReadByte(uint32_t start_addr, uint16_t cnt)
{
    return *(__IO uint8_t*)(start_addr+cnt);
}

/**********************************************************************************************************
*函 数 名: Flash_ReadWord
*功能说明: 从flash中读取一个字（32位）
*形    参: flash地址
*返 回 值: 读取到的字
**********************************************************************************************************/
uint32_t Flash_ReadWord(uint32_t addr)
{
    return *(vu32*)addr;
}


/**********************************************************************************************************
*函 数 名: Flash_GetSector
*功能说明: 获取某个地址所在的flash扇区
*形    参: flash地址
*返 回 值: addr所在的扇区（0-11）
**********************************************************************************************************/
uint16_t Flash_GetSector(uint32_t addr)
{
    if(addr<ADDR_FLASH_SECTOR_1)
        return FLASH_Sector_0;
    else if(addr<ADDR_FLASH_SECTOR_2)
        return FLASH_Sector_1;
    else if(addr<ADDR_FLASH_SECTOR_3)
        return FLASH_Sector_2;
    else if(addr<ADDR_FLASH_SECTOR_4)
        return FLASH_Sector_3;
    else if(addr<ADDR_FLASH_SECTOR_5)
        return FLASH_Sector_4;
    else if(addr<ADDR_FLASH_SECTOR_6)
        return FLASH_Sector_5;
    else if(addr<ADDR_FLASH_SECTOR_7)
        return FLASH_Sector_6;
    else if(addr<ADDR_FLASH_SECTOR_8)
        return FLASH_Sector_7;
    else if(addr<ADDR_FLASH_SECTOR_9)
        return FLASH_Sector_8;
    else if(addr<ADDR_FLASH_SECTOR_10)
        return FLASH_Sector_9;
    else if(addr<ADDR_FLASH_SECTOR_11)
        return FLASH_Sector_10;
    return FLASH_Sector_11;
}

/**********************************************************************************************************
*函 数 名: Flash_WriteByte
*功能说明: 在Flash的某个区域写入数据
*形    参: 写入地址 写入数据缓冲区指针 写入长度
*返 回 值: 写入状态
**********************************************************************************************************/
bool Flash_WriteByte(uint32_t dest,uint8_t *src,uint32_t length)
{
    FLASH_Status status = FLASH_COMPLETE;
    bool return_value = false;
    uint32_t start_addr = 0;
    uint32_t end_addr = 0;

    if(dest<FLASH_BASE_START_ADDR || dest>FLASH_BASE_END_ADDR) //非法地址
        return false;

    FLASH_Unlock();	//解锁
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
    FLASH_DataCacheCmd(DISABLE);//FLASH擦除期间，必须禁止数据缓存

    start_addr = dest;			//写入的起始地址
    end_addr = dest + length;	//写入的结束地址

    while(start_addr < end_addr)
    {
        if(Flash_ReadWord(start_addr) != 0XFFFFFFFF)//有非0XFFFFFFFF的地方，要擦除这个扇区
        {
            status = FLASH_EraseSector(Flash_GetSector(start_addr),VoltageRange_3);//VCC=2.7~3.6V之间!!

            if(status != FLASH_COMPLETE) //发生错误了
            {
                return_value = false; //发生错误了
                break;
            }
        }
        else
            start_addr+=4;
    }

    if(status == FLASH_COMPLETE)
    {
        while(dest < end_addr)//写数据
        {
            if(FLASH_ProgramByte(dest,*src) != FLASH_COMPLETE)//写入数据
            {
                return_value = false; //发生错误了
                break;
            }
            else
            {
                if (*(vu8*)dest != *src) // 检查复制到Flash的数据是否正确
                {
                    return_value = false;  // 不匹配
                    break;
                }
                else
                    return_value = true;
            }

            dest++;
            src++;
        }
    }

    FLASH_DataCacheCmd(ENABLE);	//FLASH擦除结束，开启数据缓存
    FLASH_Lock();//上锁

    return return_value;
}




