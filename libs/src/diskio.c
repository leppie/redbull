/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/

#include "diskio.h"
#include "integer.h"
#include "stm32f10x.h"
#include "stm32_eval_sdio_sd.h"

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */

SD_CardInfo cardinfo;

DSTATUS disk_initialize (
	BYTE drv				/* Physical drive nmuber (0..) */
)
{
    //SD_Error state;

    if(drv)
    {
        return STA_NOINIT;
    }

    if(!SD_Detect())
    {
        return RES_NOTRDY;
    }

/*
    state = SD_Init();

    if(state == STA_NODISK)
    {
        return STA_NODISK;
    }
    else if(state != 0)
    {
        return STA_NOINIT;
    }
    else */
    {
      NVIC_InitTypeDef NVIC_InitStructure;

      SD_GetCardInfo(&cardinfo);

      NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);

      return 0;
    }
}



/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */

DSTATUS disk_status (
	BYTE drv		/* Physical drive nmuber (0..) */
)
{
    if(drv)
    {
        return STA_NOINIT;
    }

    if(!SD_Detect())
    {
        return RES_NOTRDY;
    }
    return 0;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */

DRESULT disk_read (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	BYTE count		/* Number of sectors to read (1..255) */
)
{
  SD_Error res=0;
    if (drv || !count)
    {    
        return RES_PARERR;
    }
    if(!SD_Detect())
    {
        return RES_NOTRDY;
    }

    if(count==1)
    {                                                
        res = SD_ReadBlock(buff, sector << 9, 512);
    }                                                
    else
    {                                                
        res = SD_ReadMultiBlocks(buff, sector << 9, 512, count);
    }                                                
    if(res == SD_OK)
    {
        return RES_OK;
    }
    else
    {
        return RES_ERROR;
    }
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */

#if _READONLY == 0
DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..255) */
)
{
	SD_Error res;

    if (drv || !count)
    {    
        return RES_PARERR;
    }
    if(!SD_Detect())
    {
        return RES_NOTRDY;
    }

    if(count == 1)
    {
        res = SD_WriteBlock((uint8_t*) buff, sector << 9, 512);
    }
    else
    {
        res = SD_WriteMultiBlocks((uint8_t*) buff, sector << 9, 512, count);
    }

    if(res == 0)
    {
        return RES_OK;
    }
    else
    {
        return RES_ERROR;
    }
}
#endif /* _READONLY */



/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
    DRESULT res;
    SDTransferState status;

    if (drv)
    {    
        return RES_PARERR;
    }
    
    if(!SD_Detect())
    {
        return RES_NOTRDY;
    }

    switch(ctrl)
    {
    case CTRL_SYNC:
        status = SD_GetTransferState();
        if (status != SD_TRANSFER_BUSY)
        {
            res = RES_OK;
        }
        else
        {
            res = RES_ERROR;
        }
        break;
        
    case GET_BLOCK_SIZE:
        *(WORD*)buff = cardinfo.CardBlockSize;
        res = RES_OK;
        break;

    case GET_SECTOR_COUNT:
        *(DWORD*)buff = cardinfo.CardCapacity / cardinfo.CardBlockSize;
        res = RES_OK;
        break;
    default:
        res = RES_PARERR;
        break;
    }

    return res;
}

