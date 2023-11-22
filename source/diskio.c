/*------------------------------------------------------------------------/
/  Foolproof MMCv3/SDv1/SDv2 (in SPI mode) control module
/-------------------------------------------------------------------------/
/
/  Copyright (C) 2013, ChaN, all right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/-------------------------------------------------------------------------/
  Features and Limitations:

  * Easy to Port Bit-banging SPI
    It uses only four GPIO pins. No complex peripheral needs to be used.

  * No Media Change Detection
    Application program needs to perform a f_mount() after media change.

/-------------------------------------------------------------------------*/


#include "diskio.h"        /* Common include file for FatFs and disk I/O layer */


/*-------------------------------------------------------------------------*/
/* Platform dependent macros and functions needed to be modified           */
/*-------------------------------------------------------------------------*/

#include <project.h>
#include <cytypes.h>

/*--------------------------------------------------------------------------

   Module Private Functions

---------------------------------------------------------------------------*/
#define BLOCK_ADDRESSING_MSK 0x40000000
#define DATA_BLOCK_ACCEPTED 0x05
#define DATA_BLOCK_RES_MSK 0x1F
#define FAILED_TO_SEND_MSK 0x80
#define HCS_MSK (1UL << 30)
#define SD_V2_CSD_SHIFT 6
#define START_MSK 0x40

#define DUMMY_CRC 0x01
#define GO_IDLE_STATE_CRC 0x95
#define SD_V2_ARG 0x1AA
#define SEND_IF_COND_CRC 0x87

#define CMD_LEN 6
#define CSD_LEN 16
#define CSD_TRAILING_DATA_LEN 48
#define DUMMY_CYCLES 10
#define SECTOR_SIZE 512

#define ERASE_TIMEOUT_MS 30000
#define SD_INIT_TIMEOUT_MS 1000
#define SD_PKT_TIMEOUT_MS 200
#define SD_SPI_DELAY_MS 10
#define SD_WAIT_RDY_MS 500
#define WAIT_NUM_BYTES_MAX 20

/* MMC/SD command (SPI mode) */
typedef enum
{
    GO_IDLE_STATE = 0x00, 
    SEND_OP_COND = 0x01,
    SEND_IF_COND = 0x08,
    SEND_CSD = 0x09,
    SEND_CID = 0x0A,
    STOP_TRANSMISSION = 0x0C,
    SEND_STATUS = 0x0D,
    SET_BLOCKLEN = 0x10,
    READ_SINGLE_BLOCK = 0x11,
    READ_MULTIPLE_BLOCK = 0x12,
    SET_BLOCK_COUNT = 0x17,
    WRITE_BLOCK = 0x18,
    WRITE_MULTIPLE_BLOCK = 0x19,
    ERASE_ER_BLK_START = 0x20,
    ERASE_ER_BLK_END = 0x21,
    ERASE = 0x26,
    APP_CMD = 0x37,
    READ_OCR = 0x3A,
    SDC_SEND_STATUS = 0x80 + 13,
    SDC_WR_BLK_ERASE_COUNT = 0x80 + 23,
    SDC_SEND_OP_COND = 0x80 + 41,
} SD_SPI_CMD;

typedef enum
{
    CMD_POS = 0,
    ARG_POS = 1,
    CRC_POS = 5,
    NUM_CMD_PKT_BITS
} CMD_PKT_POS;

typedef enum
{
    TRANSMIT_BLOCK = 0xFC,
    STOP_TRAN_BLOCK = 0xFD,
    START_BLOCK = 0xFE,
    DUMMY_BYTE = 0xFF
} SD_SPI_TOKEN;

typedef enum
{
    CT_UNK = 0x00,
    CT_MMC = 0x01, // MMC v3
    CT_SD1 = 0x02, // SD v1
    CT_SD2 = 0x04, // SD v2
    CT_SDC = CT_SD1 | CT_SD2, // SD
    CT_BLOCK = 0x08 // Block addressing
} CARD_TYPE;

static DSTATUS stat = STA_NOINIT;    /* Disk status */
static CARD_TYPE card_type = CT_UNK;

static SdSpiFuncs* sd_funcs = NULL;

/**
 * @brief Waits for the SD card to be ready for proceeding
 * 
 * @param wait_ms[in] The timeout in milliseconds waiting for the SD card to be ready
 * 
 * @return true the SD card was able to be ready before the timeout
 * @return false the SD card was not able to be ready before the timeout
 */
static bool waitRdy(uint32_t wait_ms)    /* 1:OK, 0:Timeout */
{
    bool ready = false;

    sd_funcs->sd_set_timeout(wait_ms);
    while (!ready && !(sd_funcs->sd_timeout_triggered()))
    {
        if (sd_funcs->sd_xchg_fn(DUMMY_BYTE) == DUMMY_BYTE)
        {
            ready = true;
        }
    }

    return ready;
}

/**
 * @brief Deselects the SD card from SPI communication
 * 
 */
static void deselect(void)
{
    sd_funcs->deselect_sd_fn();
    sd_funcs->sd_xchg_fn(DUMMY_BYTE);
}

/**
 * @brief Selects the SD card for SPI communication and waits 
 *  for it to be ready
 * 
 * @return true the SD was selected and ready for actions
 * @return false the SD failed to be selected.
 */
static bool select(void)    /* 1:OK, 0:Timeout */
{
    bool selected = false;

    sd_funcs->select_sd_fn();
    // Force DO enabled
    sd_funcs->sd_xchg_fn(DUMMY_BYTE);

    if (waitRdy(SD_WAIT_RDY_MS))
    {
        selected = true;
    }
    else
    {
        deselect();
    }

    return selected;
}

/**
 * @brief Receive a data packet from the SD card
 * 
 * @param buff[out] the buff to store the data
 * @param btr[in] the data length
 * 
 * @return true received the data packet
 * @return false failed to receive the data packet
 */
static bool receivedDataBlock(BYTE *buff, UINT btr)
{
    bool received = false;
    BYTE token = DUMMY_BYTE;
    sd_funcs->sd_set_timeout(SD_PKT_TIMEOUT_MS);
    while ((token == DUMMY_BYTE) && !(sd_funcs->sd_timeout_triggered()))
    {
        // Insert rot_rdq() here for multitask environment.
        token = sd_funcs->sd_xchg_fn(token);
    }

    if (token == START_BLOCK)
    {
        // Store trailing data to the buffer
        sd_funcs->sd_rx_fn(buff, btr);

        // Discard CRC
        sd_funcs->sd_xchg_fn(DUMMY_BYTE);
        sd_funcs->sd_xchg_fn(DUMMY_BYTE);

        received = true;
    }

    return received;
}

/**
 * @brief Send a data packet to the SD card
 * 
 * @param buff[in] the data to send 
 * @param token[in] the command for how to handle the data
 * 
 * @return true successfully sent the data
 * @return false failed to send the data
 */
static bool sendDataBlock(const BYTE* buff, BYTE token)
{
    bool sent = false;

    if (waitRdy(SD_WAIT_RDY_MS))
    {
        sd_funcs->sd_xchg_fn(token);

        if (token != STOP_TRAN_BLOCK)
        {
            sd_funcs->sd_tx_fn(buff, SECTOR_SIZE);

            // Discard CRC
            sd_funcs->sd_xchg_fn(DUMMY_BYTE);
            sd_funcs->sd_xchg_fn(DUMMY_BYTE);

            if ((sd_funcs->sd_xchg_fn(DUMMY_BYTE) & DATA_BLOCK_RES_MSK) ==
                DATA_BLOCK_ACCEPTED)
            {
                sent = true;
            }
        }
    }

    return sent;
}

/**
 * @brief Sends the SD card command
 * 
 * @param cmd[in] the SD card command
 * @param arg[in] the argument for the command
 * 
 * @return BYTE the result from the SD card
 */
static BYTE sendCmd(BYTE cmd, DWORD arg)
{
    BYTE res = 0;

    if (cmd & FAILED_TO_SEND_MSK)
    {
        cmd &= 0x7F;
        res = sendCmd(APP_CMD, 0);
    }

    if (res <= 1)
    {
        if (cmd != STOP_TRANSMISSION)
        {
            deselect();
            if (!select())
            {
                res = 0xFF;
            }
        }
    }

    if (res <= 1)
    {
        uint8_t buf[CMD_LEN] = {
            cmd | START_MSK,
            (BYTE)(arg >> 24),
            (BYTE)(arg >> 16),
            (BYTE)(arg >> 8),
            (BYTE)arg,
            DUMMY_CRC
        };

        switch (cmd)
        {
            case GO_IDLE_STATE:
                buf[CRC_POS] = GO_IDLE_STATE_CRC;
                break;
            case SEND_IF_COND:
                buf[CRC_POS] = SEND_IF_COND_CRC;
                break;
            default:
                break;
        }
        sd_funcs->sd_tx_fn(buf, NUM_CMD_PKT_BITS);

        if (cmd == STOP_TRANSMISSION)
        {
            sd_funcs->sd_xchg_fn(DUMMY_BYTE);
        }

        for (buf[CRC_POS] = WAIT_NUM_BYTES_MAX, res = FAILED_TO_SEND_MSK;
            (res & FAILED_TO_SEND_MSK) && buf[CRC_POS];
            buf[CRC_POS]--)
        {
            res = sd_funcs->sd_xchg_fn(DUMMY_BYTE);
        }
    }

    return res;
}

/**
 * @brief Get the DWORD byte by byte since the difference with endianess
 * 
 * @return DWORD the DWORD
 */
static DWORD getDword(void)
{
    DWORD word = 0;
    for (uint8_t i = 0; i < sizeof(word); i++)
    {
        word |= sd_funcs->sd_xchg_fn(DUMMY_BYTE);
        word <<= 8;
    }

    return word;
}

/**
 * @brief Determines what card type the SD card is
 * 
 * @return CARD_TYPE the SD card type
 */
static CARD_TYPE determineCardType(void)
{
    CARD_TYPE type = CT_UNK;

    if (sendCmd(GO_IDLE_STATE, 0) == 1)
    {
        sd_funcs->sd_set_timeout(SD_INIT_TIMEOUT_MS);
        if (sendCmd(SEND_IF_COND, SD_V2_ARG) == 1)
        {
            // Checks the OCR register for the supported voltages of 2.7-3.6V
            if ((getDword() & SD_V2_ARG) == SD_V2_ARG)
            {
                // Wait for the end of initialization
                while (!(sd_funcs->sd_timeout_triggered()) &&
                    sendCmd(SDC_SEND_OP_COND, HCS_MSK));

                if (!(sd_funcs->sd_timeout_triggered()) && 
                    sendCmd(READ_OCR, 0) == 0)
                {
                    type = CT_SD2;
                    if (getDword() & BLOCK_ADDRESSING_MSK)
                    {
                        type |= CT_BLOCK;
                    }
                }
            }
            else
            {
                type = CT_SD1;
                BYTE cmd = SDC_SEND_OP_COND;
                if (sendCmd(cmd, 0) == 0)
                {
                    type = CT_MMC;
                    cmd = SEND_OP_COND;
                }
                // Wait for the end of the initialization
                while (!(sd_funcs->sd_timeout_triggered()) &&
                    sendCmd(cmd, 0));
                
                if (sd_funcs->sd_timeout_triggered() ||
                    (sendCmd(SET_BLOCKLEN, SECTOR_SIZE) != 0))
                {
                    type = CT_UNK;
                }
            }
        }
    }

    return type;
}

/**
 * @brief Gets the sector count from the SD
 * 
 * @param buff[out] the buffer to store the sector count
 * 
 * @return DRESULT non-zero result for no errors
 */
static DRESULT getSectorCnt(void* buff)
{
    DRESULT res = RES_ERROR;
    BYTE csd[CSD_LEN] = {0};

    if ((sendCmd(SEND_CSD, 0) == 0) && 
        receivedDataBlock(csd, CSD_LEN))
    {
        DWORD csize = 0;
        if ((csd[0] >> SD_V2_CSD_SHIFT) == 1)
        {
            // csize is 22 bits long in the CSD registers
            // Source: https://www.farnell.com/datasheets/1683445.pdf
            csize |= csd[7] & 0x3F;
            csize <<= 8;
            csize |= csd[8];
            csize <<= 8;
            csize |= csd[9];
            csize++;
            csize <<= 10;
        }
        else
        {
            BYTE shift = (csd[5] & 0x0F) + (csd[10] >> 7) + 
                ((csd[9] & 0x03) << 1) + 2;
            csize |= csd[6] & 0x03;
            csize <<= 8;
            csize |= csd[7];
            csize <<= 2;
            csize |= (csd[8] >> 6);
            csize++;
            csize <<= shift - 9;
        }
        *(DWORD*)buff = csize;
        res = RES_OK;
    }

    return res;
}

/**
 * @brief Gets the block size of the SD sectors
 * 
 * @param buff[out] the buffer to store the block size of the SD sectors
 * 
 * @return DRESULT non-zero result for no errors
 */
static DRESULT getBlockSize(void* buff)
{
    DRESULT res = RES_ERROR;
    BYTE csd[CSD_LEN] = {0};

    if (card_type & CT_SD2)
    {
        if (sendCmd(SDC_SEND_STATUS, 0) == 0)
        {
            sd_funcs->sd_xchg_fn(DUMMY_BYTE);

            if (receivedDataBlock(csd, CSD_LEN))
            {
                // Purge trailing data
                for (uint8_t i = CSD_TRAILING_DATA_LEN; i; i--)
                {
                    sd_funcs->sd_xchg_fn(DUMMY_BYTE);
                }

                *(DWORD*)buff = 0x10 << (csd[10] >> 4);
                res = RES_OK;
            }
        }    
    }
    else
    {
        if ((sendCmd(SEND_CSD, 0) == 0) && receivedDataBlock(csd, CSD_LEN))
        {
            DWORD block_size = 0;
            if (card_type & CT_SD1)
            {
                block_size = (csd[10] & 0x3F) << 1;
                block_size += (csd[11] >> 7) + 1;
                block_size <<= (csd[13] >> 6) - 1;
            }
            else
            {
                block_size = (csd[11] & 0x03) << 3;
                block_size += (csd[11] & 0xE0) >> 5;
                block_size++;
                block_size *= ((csd[10] & 0x7C) >> 2) + 1;
            }

            *(DWORD*)buff = block_size;
            res = RES_OK;
        }
    }

    return res;
}

/**
 * @brief Controls the erasing of the SD card data
 * 
 * @param drv[in] the drive to erase
 * @param buff[in] the start and end location for the erasing
 * 
 * @return DRESULT non-zero result for no errors
 */
static DRESULT ctrlTrim(BYTE drv, void* buff)
{
    DRESULT res = RES_ERROR;
    BYTE csd[CSD_LEN] = {0};

    if ((card_type & CT_SDC) &&
        !disk_ioctl(drv, MMC_GET_CSD, csd) &&
        ((csd[0] >> 6) || (csd[10] & 0x40)))
    {
        DWORD* data = buff;
        DWORD start = data[0];
        DWORD end = data[1];

        if (!(card_type & CT_BLOCK))
        {
            start *= SECTOR_SIZE;
            end *= SECTOR_SIZE;
        }

        if (!sendCmd(ERASE_ER_BLK_START, start) &&
            !sendCmd(ERASE_ER_BLK_END, end) &&
            !sendCmd(ERASE, 0) &&
            waitRdy(ERASE_TIMEOUT_MS))
        {
            res = RES_OK;
        }
    }

    return res;
}

/*--------------------------------------------------------------------------

   Public Functions

---------------------------------------------------------------------------*/

/**
 * @brief Gets the status of the disk
 * 
 * @param drv[in] the drive number
 * 
 * @return DSTATUS non-zero for no errors
 */
DSTATUS disk_status(BYTE drv) /* Drive number (always 0) */
{
    DSTATUS s = stat;
    if (drv) 
    {
        s = STA_NOINIT;
    }

    return s;
}

/**
 * @brief Initializes the SD card
 * 
 * @param drv[in] the drive letter
 * 
 * @return DSTATUS non-zero for no errors
 */
DSTATUS disk_initialize(BYTE drv) /* Physical drive number (0) */
{
    DSTATUS s = stat;
    if (!drv)
    {
        sd_funcs->sd_set_timeout(SD_SPI_DELAY_MS);
        while (!(sd_funcs->sd_timeout_triggered()));

        if (!(s & STA_NODISK))
        {
            sd_funcs->slow_sd_fn();
            for (uint8_t i = DUMMY_CYCLES; i; i--)
            {
                sd_funcs->sd_xchg_fn(DUMMY_BYTE);
            }

            card_type = determineCardType();
            deselect();

            s = STA_NOINIT;
            if (card_type)
            {
                sd_funcs->fast_sd_fn();
                s &= ~STA_NOINIT;
            }

            stat = s;
        }
    }
    else
    {
        s = STA_NOINIT;
    }

    return s;
}

/**
 * @brief Reads from the SD card
 * 
 * @param drv[in] the drive number
 * @param buff[out] the buffer to store the data read
 * @param sector[in] the starting sector
 * @param count[in] the number of sectors to read
 *  
 * @return DRESULT 
 */
DRESULT disk_read(BYTE drv, BYTE *buff, DWORD sector, UINT count) /* Physical drive nmuber (0) */
{
    DRESULT res = RES_OK;
    if (drv || !count)
    {
        res = RES_PARERR;
    }
    else if (stat & STA_NOINIT)
    {
        res = RES_NOTRDY;
    }
    else
    {
        if (!(card_type & CT_BLOCK))
        {
            sector *= SECTOR_SIZE;
        }

        if (count == 1)
        {
            if ((sendCmd(READ_SINGLE_BLOCK, sector) == 0) &&
                receivedDataBlock(buff, SECTOR_SIZE))
            {
                count = 0;
            }
        }
        else
        {
            if (sendCmd(READ_MULTIPLE_BLOCK, sector) == 0)
            {
                for (; count && receivedDataBlock(buff, SECTOR_SIZE); count--)
                {
                    buff += SECTOR_SIZE;
                }
                sendCmd(STOP_TRANSMISSION, 0);
            }
        }
        deselect();

        if (count != 0)
        {
            res = RES_ERROR;
        }
    }

    return res;
}

/**
 * @brief Writes to the SD card
 * 
 * @param drv[in] the drive number
 * @param buff[in] the data to write 
 * @param sector[in] the starting sector
 * @param count[in] the number of sectors to write
 * 
 * @return DRESULT non-zero for no errors
 */
DRESULT disk_write(BYTE drv, const BYTE *buff, DWORD sector, UINT count)
{
    DRESULT res = RES_OK;
    if (drv || !count)
    {
        res = RES_PARERR;
    }
    else if (stat & STA_NOINIT)
    {
        res = RES_NOTRDY;
    }
    else if (stat & STA_PROTECT)
    {
        res = RES_WRPRT;
    }
    else
    {
        if (!(card_type & CT_BLOCK))
        {
            sector *= SECTOR_SIZE;
        }

        if (count == 1)
        {
            if ((sendCmd(WRITE_BLOCK, sector) == 0) &&
                sendDataBlock(buff, START_BLOCK))
            {
                count = 0;
            }
        }
        else
        {
            if (card_type & CT_SDC)
            {
                sendCmd(SET_BLOCK_COUNT, count);
            }

            if (sendCmd(WRITE_MULTIPLE_BLOCK, sector) == 0)
            {
                for (; count && sendDataBlock(buff, TRANSMIT_BLOCK); count--)
                {
                    buff += SECTOR_SIZE;
                }

                if (!sendDataBlock(0, STOP_TRAN_BLOCK))
                {
                    res = RES_ERROR;
                }
            }
        }

        deselect();
    }

    return res;
}

/**
 * @brief Miscellaneous functions for the SD card
 * 
 * @param drv[in] the drive letter
 * @param ctrl[in] the control code 
 * @param buff[in,out] the buff for sending/storing the data
 *  
 * @return DRESULT non-zero for no errors
 */
DRESULT disk_ioctl(BYTE drv, BYTE ctrl, void* buff)
{
    DRESULT res = RES_ERROR;
    if (drv)
    {
        res = RES_PARERR;
    }
    else if (stat & STA_NOINIT)
    {
        res = RES_NOTRDY;
    }
    else
    {
        switch (ctrl)
        {
            case CTRL_SYNC:
                if (select())
                {
                    res = RES_OK;
                }
                break;
            case GET_SECTOR_COUNT:
                res = getSectorCnt(buff);
                break;
            case GET_BLOCK_SIZE:
                res = getBlockSize(buff);
                break;
            case CTRL_TRIM:
                res = ctrlTrim(drv, buff);
                break;
            default:
                res = RES_PARERR;
                break;
        }

        deselect();
    }

    return res;
}

/**
 * @brief Sets the SD card functions to use
 * 
 * @param funcs[in] the SD card functions definitions to use
 */
void setSdSpiFuncts(SdSpiFuncs* funcs)
{
    sd_funcs = funcs;
}