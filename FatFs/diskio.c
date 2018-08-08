/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */

#include <stdio.h>		/* for printf */
#include <string.h>		/* for memcpy */
#include "stm32f4xx.h"
#include "stm32f4_sdio_sd.h"


extern SD_CardInfo SDCardInfo;
BYTE SDInitialized = 0;

#define SD_DISK_PDRV	0 /* Physical drive number for SD */

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber (0..) */
)
{
	if( pdrv == SD_DISK_PDRV )
	{
		if (SDInitialized == 1)
		{
			return 0;	/* All OK */
		}

		if (SD_Init() != SD_OK)
		{
			return STA_NOINIT;
		}
		
		SDInitialized = 1;
        
		return 0;	/* All OK */
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{
	if( pdrv == SD_DISK_PDRV )
	{
		/*
		switch (SD_GetState())
		{
		case	SD_CARD_READY:
			return RES_OK;
		case	SD_CARD_IDENTIFICATION:
		case	SD_CARD_STANDBY:
		case	SD_CARD_TRANSFER:
		case	SD_CARD_SENDING:
		case	SD_CARD_RECEIVING:
		case	SD_CARD_PROGRAMMING:
		case	SD_CARD_DISCONNECTED:
			return RES_NOTRDY;
		case	SD_CARD_ERROR:
		default:
			return RES_ERROR;
		}
		*/
		
		return SDInitialized ? RES_OK : STA_NOINIT;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
#define BLOCK_SIZE 512

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	BYTE count		/* Number of sectors to read (1..128) */
)
{
//	SDTransferState SD_TransferState;
	DRESULT dresult = RES_OK;
	SD_Error SDError;
	if( pdrv == SD_DISK_PDRV )
	{	
		if(count <= 1)
		{
			SDError = SD_ReadBlock(buff, sector*BLOCK_SIZE, BLOCK_SIZE);

			#ifdef SD_DMA_MODE
			SDError = SD_WaitReadOperation();
			#endif
			
			while( SD_GetStatus() != SD_TRANSFER_OK );
			if(SDError != SD_OK)dresult = RES_ERROR;
		}
		else
		{
			SDError = SD_ReadMultiBlocksFIXED(buff, sector*BLOCK_SIZE, BLOCK_SIZE, count);
			//SDError = SD_ReadMultiBlocks(buff, sector*BLOCK_SIZE, BLOCK_SIZE, count);

			
			#ifdef SD_DMA_MODE
			SDError = SD_WaitReadOperation();
			#endif
			
			while( SD_GetStatus() != SD_TRANSFER_OK );
			if(SDError != SD_OK)dresult = RES_ERROR;
		}
		return dresult;
	}
	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..128) */
)
{
	DRESULT dresult = RES_OK;
	SD_Error SDError;
	if( pdrv == SD_DISK_PDRV )
	{	
		if(count <= 1)
		{
			SDError = SD_WriteBlock((uint8_t*)(buff), sector*BLOCK_SIZE, BLOCK_SIZE);
			
			#ifdef SD_DMA_MODE
			SDError = SD_WaitWriteOperation();
			#endif
			
			while( SD_GetStatus() != SD_TRANSFER_OK );
			if(SDError != SD_OK)dresult = RES_ERROR;
		}
		else
		{
			SDError = SD_WriteMultiBlocksFIXED((uint8_t*)(buff), sector*BLOCK_SIZE, BLOCK_SIZE, count);
			//SDError = SD_WriteMultiBlocks(buff, sector*BLOCK_SIZE, BLOCK_SIZE, count);

			#ifdef SD_DMA_MODE
			SDError = SD_WaitWriteOperation();
			#endif
			
			while( SD_GetStatus() != SD_TRANSFER_OK );
			if(SDError != SD_OK)dresult = RES_ERROR;
		}
		return dresult;
	}
	return RES_PARERR;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	if( pdrv == SD_DISK_PDRV )
	{
		
	}
	return RES_OK;
}
#endif



DWORD get_fattime ( void )
{
	return	((2013UL-1980) << 25)	      // Year = 2006
			| (8UL << 21)	      // Month = Feb
			| (30UL << 16)	      // Day = 9
			| (13U << 11)	      // Hour = 22
			| (00U << 5)	      // Min = 30
			| (00U >> 1)	      // Sec = 0
			;
}
