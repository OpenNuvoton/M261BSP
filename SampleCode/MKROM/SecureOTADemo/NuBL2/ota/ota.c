/**************************************************************************//**
 * @file     ota.c
 * @version  V1.00
 * @brief    OTA demo code
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "ota.h"

#include "NuBL_common.h"
#include "NuBL_crypto.h"
#include "NuBL2.h"

//#define printf(...)
/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8IsFwUpgradeDone = 0;
volatile uint8_t g_u8SetFwUpgradeDone = 0;
volatile uint8_t g_u8IsNuBL3xFwDone = 0;

volatile uint32_t g_u32LastSysFwWriteAddr;

extern volatile uint32_t gNuBL2_32Key[8];

volatile uint32_t g_u32FwInfoWriteBytes = 0;
FW_INFO_T g_RecvFwInfo;
FW_INFO_T g_DecryptRecvFwInfo;
volatile uint32_t g_u32NuBL3xIdentifyPass = 0;
uint32_t g_au32WriteFwBuf[44*12/4];
volatile uint8_t g_u8NuBL3xAuthSel = 0; /* Bit0:NuBL32 */
uint32_t g_au32RawData[12];   /* Decrypted Command data */
uint32_t g_au32DecryptRawData[16];   /* Decrypted temp Command data */

int32_t cmd_AES256Decrypt(uint32_t *in, uint32_t *out, uint32_t len, uint32_t *KEY, uint32_t *IV);

/* iv = "1000000000000000000000000000000a" */
uint32_t g_FwIV[4] = {0x10000000, 0x00000000, 0x00000000, 0x0000000a};

//------------------------------------------------------------------------------------------------------------------

extern void SetECCRegisters(int32_t mode, uint32_t *pPriv, uint32_t *pPub);

extern int32_t IdentifyPublicKey(uint32_t *p32Buf, int32_t mode);
extern int32_t GenCmdSessionKey(uint32_t key[]);
//------------------------------------------------------------------------------------------------------------------

ISP_INFO_T *    g_pISPInfo;

/**
  * @brief      OTA task routine
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    OTA task routine
  */
int8_t OTA_TaskProcess(void)
{
    return OTA_API_TaskProcess();
}

uint32_t Swap32(uint32_t val)
{
    return (val<<24) | ((val<<8)&0xff0000) | ((val>>8)&0xff00) | (val>>24);
}

static void NuBL_BytesSwap(char *buf, int32_t len)
{
    int32_t i;
    char    tmp;

    for(i=0; i<(len/2); i++)
    {
        tmp = buf[len-i-1];
        buf[len-i-1] = buf[i];
        buf[i] = tmp;
    }
}

/**
  * @brief Update OTA status
  * @param[in]  u8NuBL3xSel    NuBL3x secetion. BIT0: NuBL32
  * @retval     0             Success
  * @retval     others        Failed
  * @details    Update OTA status
  */
int32_t UpdateOTAStatus(uint8_t u8NuBL3xSel)
{
    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

#if (OTA_UPGRADE_FROM_SD)
    if (u8NuBL3xSel&BIT0)
    {
        FMC_Write(SYS_FW_OTA_STATUS_BASE, 1);
        if (FMC_Read(SYS_FW_OTA_STATUS_BASE) != 1)
        {
            return (-1001);
        }
    }
#else
    FMC_Write(OTA_STATUS_BASE, 1);
    if (FMC_Read(OTA_STATUS_BASE) != 1)
    {
        return (-1003);
    }
#endif

    return 0;
}

/**
  * @brief Erase NuBL32 flash region
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    Erase NuBL32 flash region
  */
/* TODO: erase size by FW INFO */
uint8_t EraseNewSysFwBlock(void)
{
    uint32_t i;
    uint32_t u32FlashPageSize;
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
    u32FlashPageSize = OTA_API_GetFlashPageSize();

    //printf("EraseNewSysFwBlock: u32FlashPageSize=0x%x(%d)\n", u32FlashPageSize, u32FlashPageSize);

    for(i = 0U; i < (uint32_t)SYS_NEW_FW_BLOCK_SIZE; i+=u32FlashPageSize)
    {
        /* Erase page */
        if (OTA_API_EraseFlash((uint32_t)SYS_FW_BASE + i) != 0U)
        {
            printf("Erase fail(0x%x)\n", (uint32_t)SYS_FW_BASE+i);
            return STATUS_FAILED;
        }
    }

    printf("\nerase new system firmware block has done\n");

    return STATUS_SUCCESS;
}

/**
  * @brief      Get flag of firmware upgrade done
  * @param      None
  * @return     Flag of firmware upgrade done
  * @details    This function is to get flag of firmware upgrade done.
  */
uint8_t OTA_GetFwUpgradeDone(void)
{
    return g_u8IsFwUpgradeDone;
}


/**
  * @brief Write data to flash
  * @param[in]  u32Address    Flash address
  * @param[in]  pu8Buff       The pointer of data buffer
  * @param[in]  u32Size       Data size
  * @retval     0             Success
  * @retval     others        Failed
  * @details    Write data to flash
  */
uint8_t OTA_WriteNewFW(uint32_t u32Address, uint8_t* pu8Buff, uint32_t u32Size)
{
    uint8_t u8Status = STATUS_FAILED;
    uint16_t u16Idx;

    //DEBUG_MSG("OTA_WriteNewSysFW: addr 0x%x ~ ", u32Address);
    for (u16Idx = 0U; u16Idx < u32Size; u16Idx += 4U)
    {
        if (OTA_API_WriteFlash(u32Address, (pu8Buff[u16Idx+3]<<24) | (pu8Buff[u16Idx+2]<<16) | (pu8Buff[u16Idx+1]<<8) | (pu8Buff[u16Idx])))
            u8Status = STATUS_FAILED;
        else
            u8Status = STATUS_SUCCESS;

        if (u8Status != STATUS_SUCCESS)
            break;

        u32Address += 4U;
        g_u32LastSysFwWriteAddr = u32Address;
    }

    return u8Status;
}


/**
  * @brief      Compare NuBL32 F/W Version
  * @param[in]  u32FwVer    F/W version of Remote NuBL32  \n
  * @param[in]  u32Mode     bit-0: 0:verify NuBL32 \n
  * @retval     0           Is Newer
  * @retval     others      Is Older or Failed
  * @details    This function is used to compare if remote F/W version of NuBL32 is newer than local one. \n
  *             Flow: \n
  *                 1. Get NuBL3x info \n
  *                 2. Compare NuBL3x F/W version (enclosed in F/W info) \n
  */
int32_t NuBL2_CompareNuBL3xVer(uint32_t u32FwVer, int32_t u32Mode)
{
    volatile int32_t    i, ret = -1000;
    uint32_t            *infobuf, base, len;
    FW_INFO_T           FwInfo;

    if(!(u32Mode == 0))
    {
        NUBL_MSG("\nCompare NuBL3x Version FAIL. Invalid mode: 0x%x.\n\n", u32Mode);
        return ret;
    }

    NUBL_MSG("\nCompare NuBL32. \n\n");

    memset(&FwInfo, 0x0, sizeof(FW_INFO_T));

    infobuf = (uint32_t *)&FwInfo;

/* Step 1. Get NuBL3x info */
    /* Get NuBL3x F/W info */
    len = sizeof(FW_INFO_T);
    if((u32Mode&BIT4) != BIT4)
    {
        if((u32Mode&BIT0) == 0)
            base = NUBL32_FW_INFO_BASE;   // encrypted NuBL32 info address

        for(i=0; i<(len/4); i++)
            infobuf[i] = FMC_Read(base + (i*4));
    }
    NUBL_MSG("Get NuBL32 F/W info [Done]\n\n");

/* Step 2. Compare NuBL3x F/W version (enclosed in F/W info) */
    memcpy(&FwInfo, infobuf, sizeof(FW_INFO_T));
    if (u32FwVer <= FwInfo.mData.au32ExtInfo[0])
    {
        NUBL_MSG("Remote NuBL32 F/W version is [Older]\n\n");
        ret = -3001;
        goto _exit_NuBL2_CompareNuBL3xVer;
    }
    NUBL_MSG("Remote NuBL32 F/W version is [Newer]\n\n");

    ret = 0;

_exit_NuBL2_CompareNuBL3xVer:
    SYS_ResetModule(CRPT_RST);

    memset(&FwInfo, 0x0, sizeof(FW_INFO_T));

    return ret;
}

/**
  * @brief      Check write space for NuBL32 F/W
  * @param[in]  u32FwSize   check free spcace for writing new NuBL32  \n
  * @param[in]  u8Mode      bit-0: 0:Check NuBL32 \n
  * @retval     0           Is enough
  * @retval     others      no enough space or Failed
  * @details    This function is used to check if local space is enough for new F/W size of NuBL32. \n
  */
int8_t CheckNuBL3xWriteSpace(uint32_t u32FwSize, uint8_t u8Mode)
{
    volatile int32_t i, ret = -1000;
    uint32_t u32ReservedSize;

    if(!(u8Mode == 0))
    {
        DEBUG_MSG("\nCheck NuBL3x firmware write space FAIL. Invalid mode: 0x%x.\n\n", u8Mode);
        return ret;
    }

    if((u8Mode&BIT0) == 0)
        /* Get reserved NuBL32 flash size */
        u32ReservedSize = NUBL32_LIB_BASE - NUBL32_FW_BASE;

    if (u32FwSize > u32ReservedSize)
    {
        /* Reserved flash space is not enough for new firmware */
        DEBUG_MSG("\nCheck NuBL32 firmware write space is not enough. %d : %d\n\n", u32FwSize,u32ReservedSize);
        ret = -1001;
    }

    return 0;
}

/**
  * @brief Check write address and size is valid
  * @param[in]  addr   Flash address
  * @param[in]  size   write size
  * @retval     0           Success
  * @retval     others      Failed
  * @details    Check write address and size is valid in system.
  */
static int32_t _IsValidFlashRegion(uint32_t addr, uint32_t size)
{
    uint32_t u32APROMSize = FMC_APROM_END - FMC_APROM_BASE;
    DEBUG_MSG("[Check flash region] addr: 0x%x, size: %d.\n", addr, size);

    /* Check address and length */
    if((addr%4) != 0)
        return ERR_INVALID_ADDRESS;

    if(((size%4) != 0) || (size == 0))
        return ERR_INVALID_ADDRESS;

    if((addr < u32APROMSize) && (addr >= FMC_APROM_BASE))
    {
        if(((addr+size) > u32APROMSize) || ((addr+size) < addr))
            return ERR_OVER_RANGE;
    }
    else if((addr < FMC_LDROM_END) && (addr >= FMC_LDROM_BASE))
    {
        return ERR_OVER_RANGE;
    }
    else
    {
        return ERR_OVER_RANGE;
    }

    return 0;
}

#if (OTA_UPGRADE_FROM_SD)
/**
  * @brief Verify NuBL32 firmware integrity.
  * @param[in]  u8BLxSel    NuBL3x selection. Bit0 = 1: NuBL32
  * @param[in]  pu32Key3x   The pointer of NuBL3x Key.
  * @retval     0           Success
  * @retval     others      Failed
  * @details    NuBL3x firmware integrity verification from SD card.
  */
int8_t VerifyNuBL3xIntegrityFromSD(uint8_t u8BLxSel, uint32_t pu32Key3x[])
{
    uint32_t u32ReadBufLen, i8Ret = 0, u32FwInfoSize, u32ReadLen, u32DecryptDataLen;
    uint32_t au32HashBuf[8], au32RawData[12], au32ReadBuf[12], au32DecryptRawData[16];
    uint32_t u32ShaState;
    uint8_t u8NotFirstRead;
    uint32_t u32FwSize, start, end, len;
    uint32_t * pu32ReadBuf;
    FW_INFO_T FwInfoTmp;

    u8NotFirstRead = 0;
    /* Get new FW INFO from SD card */
    u32FwInfoSize = sizeof(FwInfoTmp);
    memset((uint8_t *)&FwInfoTmp, 0, u32FwInfoSize);
    /* Get new FW INFO from SD */
    /* Open the firmware package file */
    OTA_API_SDFwPackReadOpen(u8BLxSel);
    /* Read the FW INFO data from firmware package file */
    OTA_API_SDRead((uint8_t *)&FwInfoTmp, u32FwInfoSize, &u32ReadLen);
    if (u32FwInfoSize != u32ReadLen)
    {
        printf("VerifyNuBL3xIntegrityFromSD: read NuBL32 FW INFO length error!\n");
        return (-1);
    }

    /* Set read buffer of firmware package data */
    pu32ReadBuf = au32ReadBuf;
    /* Set read buffer length */
    u32ReadBufLen = sizeof(au32RawData);
    /* Set total read length by firmware region in FW INFO */
    if ((FwInfoTmp.mData.au32FwRegion[1].u32Start == 0x0)&&(FwInfoTmp.mData.au32FwRegion[1].u32Size == 0x0))
    {
        /* Only one valid firmware region in FW INFO */
        u32FwSize = FwInfoTmp.mData.au32FwRegion[0].u32Size;
    }
    else
    {
        /* Two valid firmware region in FW INFO */
        u32FwSize = FwInfoTmp.mData.au32FwRegion[0].u32Size + FwInfoTmp.mData.au32FwRegion[1].u32Size;
    }
    /* NuBL32 FW integrity check */
    u32ShaState = 0;
    u32ReadLen = u32ReadBufLen;
    u32DecryptDataLen = sizeof(au32DecryptRawData);
    while(u32ReadBufLen <= u32ReadLen)
    {
        /* Read encrypted firmware data */
        if (OTA_API_SDRead((uint8_t *)pu32ReadBuf, u32ReadBufLen, &u32ReadLen) != 0)
        {
            printf("VerifyNuBL3xIntegrityFromSD: read NuBL32 FW error!\n");
            i8Ret = (-2);
        }

        if (u32ReadBufLen != u32ReadLen)
        {
            printf("VerifyNuBL3xIntegrityFromSD: read NuBL32 FW last data!(u32ReadLen:%d)\n", u32ReadLen);
            if(u32ReadLen)
            {
                if (u8NotFirstRead == 0)
                {
                    /* First Read Data */
                    memcpy((uint8_t *)&au32RawData, pu32ReadBuf, u32ReadBufLen);
                    /* Decrypt new firmware data */
                    cmd_AES256Decrypt(au32RawData, au32RawData, u32ReadBufLen, pu32Key3x, g_FwIV);
                    /* CFB mode need bofore 16 bytes to be decrypted current raw data correctly. */
                    memcpy((uint8_t *)&au32DecryptRawData, pu32ReadBuf+8, 16);
                    u8NotFirstRead = 1;
                }
                else
                {
                    /* Not first read data, so CFB mode need bofore 16 bytes to be decrypted current raw data correctly. */
                    memcpy((uint8_t *)&au32DecryptRawData + 16, pu32ReadBuf, u32ReadBufLen);
                    /* Decrypt new firmware data */
                    cmd_AES256Decrypt(au32DecryptRawData, au32DecryptRawData, u32DecryptDataLen, pu32Key3x, g_FwIV);
                    memcpy((uint8_t *)&au32RawData, (uint8_t *)&au32DecryptRawData + 16, u32ReadBufLen);
                    /* CFB mode need bofore 16 bytes to be decrypted current raw data correctly. */
                    memcpy((uint8_t *)&au32DecryptRawData, pu32ReadBuf+8, 16);
                }

                if (u32ShaState)
                {
                    if (u32FwSize <= u32ReadLen)
                    {
                        /* End of calculate hash */
                        start = (uint32_t )&au32RawData[0];
                        len = u32FwSize;
                        end   = start + len;
                        NuBL_CalculateSHA256(start, end, (uint32_t *)&au32HashBuf[0], SHA_CONTI_END, SHA_SRC_SRAM);
                        break;
                    }
                    else
                        printf("[WARN] Hash value was incorrect by FW size is not matched\n");
                }
            }
            else
            {
                printf("[WARN] Read length is 0 !\n");
                //while(1){}
            }
        }
        else
        {
            if(u32ReadLen)
            {
                if (u8NotFirstRead == 0)
                {
                    /* First Read Data */
                    memcpy((uint8_t *)&au32RawData, pu32ReadBuf, u32ReadBufLen);
                    /* Decrypt new firmware data */
                    cmd_AES256Decrypt(au32RawData, au32RawData, u32ReadBufLen, pu32Key3x, g_FwIV);
                    /* CFB mode need bofore 16 bytes to be decrypted current raw data correctly. */
                    memcpy((uint8_t *)&au32DecryptRawData, pu32ReadBuf+8, 16);
                    u8NotFirstRead = 1;
                }
                else
                {
                    /* Not first read data, so CFB mode need bofore 16 bytes to be decrypted current raw data correctly. */
                    memcpy((uint8_t *)&au32DecryptRawData + 16, pu32ReadBuf, u32ReadBufLen);
                    /* Decrypt new firmware data */
                    cmd_AES256Decrypt(au32DecryptRawData, au32DecryptRawData, u32DecryptDataLen, pu32Key3x, g_FwIV);
                    memcpy((uint8_t *)&au32RawData, (uint8_t *)&au32DecryptRawData + 16, u32ReadBufLen);
                    /* CFB mode need bofore 16 bytes to be decrypted current raw data correctly. */
                    memcpy((uint8_t *)&au32DecryptRawData, pu32ReadBuf+8, 16);
                }

                if (u32ShaState == 0)
                {
                    /* Start of calculate hash */
                    start = (uint32_t )&au32RawData[0];
                    len = u32ReadLen;
                    end   = start + len;
                    NuBL_CalculateSHA256(start, end, (uint32_t *)&au32HashBuf[0], SHA_CONTI_START, SHA_SRC_SRAM);
                }
                else
                {
                    if (u32FwSize <= u32ReadLen)
                    {
                        /* End of calculate hash */
                        start = (uint32_t )&au32RawData[0];
                        len = u32FwSize;
                        end   = start + len;
                        NuBL_CalculateSHA256(start, end, (uint32_t *)&au32HashBuf[0], SHA_CONTI_END, SHA_SRC_SRAM);
                        break;
                    }
                    else
                    {
                        /* Continuous of calculate hash */
                        start = (uint32_t )&au32RawData[0];
                        len = u32ReadLen;
                        end   = start + len;
                        NuBL_CalculateSHA256(start, end, (uint32_t *)&au32HashBuf[0], SHA_CONTI_ING, SHA_SRC_SRAM);
                    }
                }
                u32FwSize -= u32ReadLen;
            }
        }
        u32ShaState++;
    }
    /* Close firmware package file */
    OTA_API_SDClose(u8BLxSel);
    u8NotFirstRead = 0;

    /* Verify NuBL3x FW hash */
    if(memcmp(&au32HashBuf, FwInfoTmp.au32FwHash, sizeof(au32HashBuf)) != 0)
    {
        NUBL_MSG("Identify NuBL32 FW Hash [FAIL]\n\n");
        return (-3);
    }

    NUBL_MSG("Identify NuBL32 FW Hash [PASS]\n\n");

    return 0;
}

/**
  * @brief Update NuBL32 firmware
  * @param[in]  u8BLxSel    NuBL3x selection. Bit0 = 1: NuBL32
  * @param[in]  pu32Key3x   The pointer of NuBL3x Key.
  * @retval     0           Success
  * @retval     others      Failed
  * @details    Update NuBL3x firmware from SD card(the firmware package file).
  */
int8_t UpdateNuBL3xFwFromSD(uint8_t u8BLxSel, uint32_t pu32Key3x[])
{
    uint32_t u32ReadBufLen, i8Ret = 0, u32FwInfoSize, u32ReadLen, u32DecryptDataLen, u32Fw1Size, u32Fw1Start, u32LastWriteAddr;
    uint32_t u32Fw2Start, u32Fw2Size;
    uint32_t au32HashBuf[8], au32RawData[12], au32ReadBuf[12], au32DecryptRawData[16];
    uint8_t u8NotFirstRead;
    uint32_t * pu32ReadBuf;
    FW_INFO_T FwInfoTmp;

    u8NotFirstRead = 0;
    u32FwInfoSize = sizeof(FwInfoTmp);
    memset((uint8_t *)&FwInfoTmp, 0, u32FwInfoSize);
    /* Get new FW INFO from SD */
    /* Open the firmware package file */
    OTA_API_SDFwPackReadOpen(u8BLxSel);
    /* Read the FW INFO data from firmware package file */
    OTA_API_SDRead((uint8_t *)&FwInfoTmp, u32FwInfoSize, &u32ReadLen);
    if (u32FwInfoSize != u32ReadLen)
    {
        printf("UpdateNuBL3xFwFromSD: read NuBL3%d FW INFO length error!\n",((u8BLxSel&BIT0)==1)?2:3);
        return (-1);
    }

    /* Get start address and size of new firmware from FW INFO */
    u32LastWriteAddr = FwInfoTmp.mData.au32FwRegion[0].u32Start;
    u32Fw1Size = FwInfoTmp.mData.au32FwRegion[0].u32Size;
    u32Fw1Start = FwInfoTmp.mData.au32FwRegion[0].u32Start;
    u32Fw2Size = FwInfoTmp.mData.au32FwRegion[1].u32Size;
    u32Fw2Start = FwInfoTmp.mData.au32FwRegion[1].u32Start;

    u32ReadBufLen = sizeof(au32RawData);
    /* Set read buffer of firmware package data */
    pu32ReadBuf = au32ReadBuf;
    u32ReadLen = u32ReadBufLen;
    u32DecryptDataLen = sizeof(au32DecryptRawData);
    while(u32ReadBufLen <= u32ReadLen)
    {
        /* Read the new firmware data from firmware package file */
        if (OTA_API_SDRead((uint8_t *)pu32ReadBuf, u32ReadBufLen, &u32ReadLen) != 0)
        {
            printf("UpdateNuBL3xFwFromSD: read NuBL3%d FW error!\n",((u8BLxSel&BIT0)==1)?2:3);
            return (-2);
        }
        if (u32ReadBufLen != u32ReadLen)
        {
            printf("UpdateNuBL3xFwFromSD: read NuBL3%d FW last data!(u32ReadLen:%d)\n",((u8BLxSel&BIT0)==1)?2:3, u32ReadLen);
        }
        if(u32ReadLen)
        {
            if (u8NotFirstRead == 0)
            {
                /* First Read Data */
                memcpy((uint8_t *)&au32RawData, pu32ReadBuf, u32ReadBufLen);
                /* Decrypt new firmware data */
                cmd_AES256Decrypt(au32RawData, au32RawData, u32ReadBufLen, pu32Key3x, g_FwIV);
                /* CFB mode need bofore 16 bytes to be decrypted current raw data correctly. */
                memcpy((uint8_t *)&au32DecryptRawData, pu32ReadBuf+8, 16);
                u8NotFirstRead = 1;
            }
            else
            {
                /* Not first read data, so CFB mode need bofore 16 bytes to be decrypted current raw data correctly. */
                memcpy((uint8_t *)&au32DecryptRawData + 16, pu32ReadBuf, u32ReadBufLen);
                /* Decrypt new firmware data */
                cmd_AES256Decrypt(au32DecryptRawData, au32DecryptRawData, u32DecryptDataLen, pu32Key3x, g_FwIV);
                memcpy((uint8_t *)&au32RawData, (uint8_t *)&au32DecryptRawData + 16, u32ReadBufLen);
                /* CFB mode need bofore 16 bytes to be decrypted current raw data correctly. */
                memcpy((uint8_t *)&au32DecryptRawData, pu32ReadBuf+8, 16);
            }

            /* Check last write address is over FW1 start + FW1 size */
            if(((u32LastWriteAddr) + u32ReadLen) > u32Fw1Start + u32Fw1Size)
            {
                uint32_t u32OverFwSize;

                /* Calculate Fw2 size */
                u32OverFwSize = (u32LastWriteAddr) + u32ReadLen - (u32Fw1Start + u32Fw1Size);
                /* Check if last write address is more the payload size of one package than FW1 range */
                if (u32OverFwSize < u32ReadBufLen)
                {
                    /* Write new firmware data to flash */
                    if (OTA_WriteNewFW(u32LastWriteAddr, (uint8_t *)&au32RawData, u32ReadLen - u32OverFwSize) != STATUS_SUCCESS)
                    {
                        printf("UpdateNuBL3xFwFromSD: write new NuBL3%d FW to flash error\n",((u8BLxSel&BIT0)==1)?2:3);
                        return (-3);
                    }
                    /* Check firmware size is not 0 in FW INFO decription */
                    if (u32Fw2Size != 0)
                    {
                        /* Check write address of FW INFO is valid in system */
                        if((i8Ret =_IsValidFlashRegion(u32Fw2Start, u32Fw2Size)) != 0)
                            return i8Ret;

                        g_u32LastSysFwWriteAddr = u32Fw2Start;
                        if (u32LastWriteAddr < u32Fw2Start)
                            u32LastWriteAddr = g_u32LastSysFwWriteAddr;
                        /* Write new firmware data to flash */
                        if (OTA_WriteNewFW(u32LastWriteAddr, (uint8_t *)&au32RawData + (u32ReadLen - u32OverFwSize), u32OverFwSize) != STATUS_SUCCESS)
                        {
                            printf("UpdateNuBL3xFwFromSD: write new NuBL3%d FW to flash error\n",((u8BLxSel&BIT0)==1)?2:3);
                            return (-5);
                        }
                    }
                }
                else
                {
                    if (u32OverFwSize == u32ReadBufLen)
                    {
                        /* Check firmware size is not 0 in FW INFO decription */
                        if (FwInfoTmp.mData.au32FwRegion[1].u32Size != 0)
                        {
                            /* Check write address of FW INFO is valid in system */
                            if((i8Ret =_IsValidFlashRegion(u32Fw2Start, u32Fw2Size)) != 0)
                                return i8Ret;

                            u32LastWriteAddr = u32Fw2Start;
                        }
                    }
                    /* Write new firmware data to flash */
                    if(((g_u32LastSysFwWriteAddr) + u32ReadLen) < u32Fw2Start + u32Fw2Size)
                    {
                        if (OTA_WriteNewFW(g_u32LastSysFwWriteAddr, (uint8_t *)&au32RawData, u32ReadLen) != STATUS_SUCCESS)
                        {
                            printf("UpdateNuBL3xFwFromSD: write new NuBL3%d FW to flash error\n",((u8BLxSel&BIT0)==1)?2:3);
                            return (-7);
                        }
                    }
                    else
                    {
                        if (u32Fw2Start + u32Fw2Size - (g_u32LastSysFwWriteAddr))
                        {
                            if (OTA_WriteNewFW(g_u32LastSysFwWriteAddr, (uint8_t *)&au32RawData, u32Fw2Start + u32Fw2Size - (g_u32LastSysFwWriteAddr)) != STATUS_SUCCESS)
                            {
                                printf("UpdateNuBL3xFwFromSD: write new NuBL3%d FW to flash error\n",((u8BLxSel&BIT0)==1)?2:3);
                                return (-7);
                            }
                        }
                    }
                }
            }
            else
            {
                /* Write new firmware data to flash */
                if (OTA_WriteNewFW(u32LastWriteAddr, (uint8_t *)au32RawData, u32ReadLen) != STATUS_SUCCESS)
                {
                    printf("UpdateNuBL3xFwFromSD: write new NuBL3%d FW to flash error\n",((u8BLxSel&BIT0)==1)?2:3);

                    return (-8);
                }
            }
        }
        u32LastWriteAddr += u32ReadLen;
    }
    /* Close firmware package file */
    OTA_API_SDClose(u8BLxSel);
    u8NotFirstRead = 0;

    return 0;
}

/**
  * @brief Verify and Update NuBL32 from SD card.
  * @param[in]  u8BLxSel    NuBL3x selection. Bit0 = 1: NuBL32
  * @retval     0           Success
  * @retval     others      Failed
  * @details    NuBL3x identity, authentication and firmware integrity.
  *             Update NuBL3x FW INFO and firmware.
  */
int32_t OTA_VerifyAndUpdateNuBL3xFromSD(uint8_t u8BLxSel)
{
    uint32_t u32FwSize, u32FwInfoSize = 0, u32ReadLen, u32FwInfoBase;
    int32_t i32Mode = 0, i32Status;
    uint32_t * pNuBL3xKey;
    uint32_t * pu32ReadBuf;
    FW_INFO_T FwInfoTmp;

    i32Status = 0;

    /* Get ECDH key for decrypt NuBL3x firmware */
    NuBL2_GetNuBL3xECDHKeys((uint32_t *)gNuBL2_32Key, NULL);

    /* Initial local variables for NuBL32 */
    if (u8BLxSel == BIT0)
    {
        /* Initial local variables for NuBL32 */
        i32Mode = 0;
        pNuBL3xKey = (uint32_t *)&gNuBL2_32Key;
    }

    /* Get new FW INFO from SD card */
    u32FwInfoSize = sizeof(FwInfoTmp);
    memset((uint8_t *)&FwInfoTmp, 0, u32FwInfoSize);
    /* Open the firmware package file */
    OTA_API_SDFwPackReadOpen(u8BLxSel);
    /* Read the FW INFO data from firmware package file */
    OTA_API_SDRead((uint8_t *)&FwInfoTmp, u32FwInfoSize, &u32ReadLen);
    if (u32FwInfoSize != u32ReadLen)
    {
        printf("OTA_VerifyAndUpdateNuBL3xFromSD: read NuBL32 FW INFO length error!\n");
        return (-1);
    }
    /* Close firmware package file */
    OTA_API_SDClose(u8BLxSel);

    /* Verify FW INFO */
    if (NuBL2_ExecuteVerifyNuBL3x((uint32_t *)&FwInfoTmp, 0x10|i32Mode) != 0)
    {
        printf("OTA_VerifyAndUpdateNuBL3xFromSD: identify new NuBL32 FW package error!\n");
        return (-2);
    }
    /* Verify NuBL3x FW hash */
    if (VerifyNuBL3xIntegrityFromSD(u8BLxSel, pNuBL3xKey) != 0 )
    {
        printf("OTA_VerifyAndUpdateNuBL3xFromSD: Verify NuBL32 FW Hash [FAIL]\n\n");
        return (-3);
    }
    printf("Identify NuBL32 FW Hash [PASS]\n\n");

    /* Get total flash size for new NuBL3x firmware */
    if ((FwInfoTmp.mData.au32FwRegion[1].u32Start == 0x0)&&(FwInfoTmp.mData.au32FwRegion[1].u32Size == 0x0))
    {
        /* Only one valid firmware region in FW INFO */
        u32FwSize = FwInfoTmp.mData.au32FwRegion[0].u32Size;
    }
    else
    {
        /* Two valid firmware region in FW INFO */
        u32FwSize = FwInfoTmp.mData.au32FwRegion[0].u32Size + FwInfoTmp.mData.au32FwRegion[1].u32Size;
    }
    /* Check NuBL3x flash size is enough to write. */
    if (CheckNuBL3xWriteSpace(u32FwSize ,i32Mode) != 0)
    {
        /* NuBL3x flash size is not enough for new firmware */
        printf("OTA_VerifyAndUpdateNuBL3xFromSD: NuBL32 flash size is not enough!\n");
        i32Status = (-4);
    }

    /* Set flash address of NuBL32 FW INFO */
    if (i32Mode == 0)
        u32FwInfoBase = NUBL32_FW_INFO_BASE;

    /* Update FW INFO */
    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
    /* Update NuBL32 FW INFO to flash */
    if (NuBL2_UpdateNuBL3xFwInfo((uint32_t *)&FwInfoTmp, sizeof(FW_INFO_T), i32Mode, u32FwInfoBase) != 0)
    {
        printf("OTA_VerifyAndUpdateNuBL3xFromSD: NuBL32 FW INFO write error\n");
        i32Status = (-5);
    }

    if(i32Mode == 0)
    {
        /* Erase old NuBL32 */
        EraseNewSysFwBlock();
    }

    /* Update NuBL32 firmware */
    if (UpdateNuBL3xFwFromSD(u8BLxSel, pNuBL3xKey) != 0)
    {
        printf("OTA_VerifyAndUpdateNuBL3xFromSD: Update NuBL32 firmware was failed\n");
        i32Status = (-6);
    }

    return i32Status;
}
#endif

/**
  * @brief Generate packet of response command and send out
  * @param[in]  pCmd           The pointer of response command
  * @param[in]  u16PacketID    The packet ID of response command
  * @param[in]  pISPInfo       The pointer of ISP Info
  * @param[in]  u32Status      Command process Status
  * @retval     0              Success
  * @retval     others         Failed
  * @details    Generate packet of response command include checksum calculation, and send it out.
  */
int32_t OTA_GenRspPacket(CMD_PACKET_T * pCmd, uint16_t u16PacketID, ISP_INFO_T *pISPInfo, uint32_t u32Status)
{
    CMD_PACKET_T        cmd;

    //memset(&cmd, 0x0, sizeof(CMD_PACKET_T));
    memcpy(&cmd, pCmd, sizeof(CMD_PACKET_T));

    memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
    /* Fill command content by input parameters */
    cmd.u16PacketID = u16PacketID;
    cmd.au32Data[0] = u32Status;
    cmd.u16Len      = (4 * 1);
    DEBUG_MSG("OTA_GenRspPacket(PID:%d)\n", cmd.u16PacketID);

    /* Checksum calculation for command packet */
    Cmd_GenRspPacket(&cmd, pISPInfo);
    /* Copy packet to response buffer of ISP Info */
    memcpy(pISPInfo->rspbuf, &cmd, sizeof(CMD_PACKET_T));
    /* Send command packet */
    OTA_API_SendFrame((uint8_t *) pISPInfo->rspbuf, MAX_FRAME_SIZE);

    return 0;
}

/**
  * @brief Set public key 0 of NuBL2
  * @param[in]  pub0   The pointer of public key
  * @return     None
  * @details    Set public key 0 of NuBL2.
  */
//const char cNuBL2_pub0[] = "755b3819f05a3e9f32d4d599062834aac5220f75955378414a8f63716a152ce2";
static void SetPubKey0(uint32_t *pub0)
{
    pub0[0] = 0x19385b75;
    pub0[1] = 0x9f3e5af0;
    pub0[2] = 0x99d5d432;
    pub0[3] = 0xaa342806;
    pub0[4] = 0x750f22c5;
    pub0[5] = 0x41785395;
    pub0[6] = 0x71638f4a;
    pub0[7] = 0xe22c156a;
}

/**
  * @brief Set public key 1 of NuBL2
  * @param[in]  pub0   The pointer of public key
  * @return     None
  * @details    Set public key 1 of NuBL2.
  */
//const char cNuBL2_pub1[] = "91c413f1915ed7b47473fd797647ba3d83e8224377909af5b30c530eaad79fd7";
static void SetPubKey1(uint32_t *pub1)
{
    pub1[0] = 0xf113c491;
    pub1[1] = 0xb4d75e91;
    pub1[2] = 0x79fd7374;
    pub1[3] = 0x3dba4776;
    pub1[4] = 0x4322e883;
    pub1[5] = 0xf59a9077;
    pub1[6] = 0x0e530cb3;
    pub1[7] = 0xd79fd7aa;
}

/**
  * @brief      Initial Secure ISP
  * @param      pISPInfo    The pointer of ISP Info
  * @retval     0           Success
  * @retval     -1          Failed
  * @details    This function initial the Secure ISP to update F/W.
  * @note       initial ClientPubKey and ServerPubKey is identity public key,
                but both key values will be changed to random public keys after ECDH key exchanged done.
  */
int32_t IspInfo_Init(ISP_INFO_T *pISPInfo)
{
    uint32_t    priv[8], msg[8], R[8], S[8], AESKey[8];
    uint32_t    start, end, i;

    //memset((void *)&g_ISPInfo, 0x0, sizeof(ISP_INFO_T));
    memset((void *)pISPInfo, 0x0, sizeof(ISP_INFO_T));

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_ENABLE_ISP();

    /* Enable all interrupt */
    __set_PRIMASK(0);
    /* Configure global SysInfo */
    SetPubKey0(pISPInfo->ClientPubKey.au32Key0);
    SetPubKey1(pISPInfo->ClientPubKey.au32Key1);

    /* Generate IV for package encrypt */
    msg[0] = SYS->PDID;
    msg[1] = BL_ReadUID(0);
    msg[2] = BL_ReadUID(1);
    msg[3] = BL_ReadUID(2);
    msg[4] = BL_ReadUCID(0);
    msg[5] = BL_ReadUCID(1);
    msg[6] = BL_ReadUCID(2);
    msg[7] = BL_ReadUCID(3);
    start = (uint32_t )msg;
    end   = start + sizeof(msg);
    NuBL_CalculateSHA256(start, end, msg, SHA_ONESHOT, SHA_SRC_SRAM);
    BL_GetIDECCSignature(R, S);
    memcpy(pISPInfo->sign.au32R, R, sizeof(R));
    memcpy(pISPInfo->sign.au32S, S, sizeof(S));
    /* Copy IV to ISP Info */
    memcpy(pISPInfo->au32AESIV, R, (128/8));
    /* Change endian for IV */
    pISPInfo->au32AESIV[0] = __REV(pISPInfo->au32AESIV[0]); // ByteSwap32
    pISPInfo->au32AESIV[1] = __REV(pISPInfo->au32AESIV[1]); // ByteSwap32
    pISPInfo->au32AESIV[2] = __REV(pISPInfo->au32AESIV[2]); // ByteSwap32
    pISPInfo->au32AESIV[3] = __REV(pISPInfo->au32AESIV[3]); // ByteSwap32

    pISPInfo->u32CmdMask = 0;

    return 0;
}

/**
  * @brief OTA process initialization
  * @param[in]  u32HSI      PLL Output Clock Frequency
  * @param[in]  pISPInfo    The pointer of ISP Info
  * @retval     0           Success
  * @retval     others      Failed
  * @details    OTA process initialization
  */
int32_t OTA_Init(uint32_t u32HSI, ISP_INFO_T *pISPInfo)
{
    int32_t i32Ret;

    printf("OTA_Init\n");

    i32Ret = 0;
    SYS_UnlockReg();

    /* Set the global ISP Info pointer */
    g_pISPInfo = pISPInfo;

    /* This global structure need to be initialized before re-connect, because client and server key was changed to random public key. */
    //IspInfo_Init((ISP_INFO_T *)&g_ISPInfo);
    i32Ret = IspInfo_Init(pISPInfo);
    if (i32Ret)
    {
        printf("OTA_Init: ISPInfo initial failed!\n");
        return i32Ret;
    }

    g_u32LastSysFwWriteAddr = SYS_FW_BASE;

    /* Init CyclesPerUs value for system tick */
    OTA_API_Init(u32HSI);

#if (OTA_UPGRADE_FROM_SD)
    /* Init SD Host */
    OTA_API_SDInit();
#endif

    return i32Ret;
}

/**
  * @brief OTA system tick interrupt process
  * @param[in]  u32Ticks    Tick value
  * @retval     0           Success
  * @retval     others      Failed
  * @details    OTA system tick interrupt process
  */
uint8_t OTA_SysTickProcess(uint32_t u32Ticks)
{
    return OTA_API_SysTickProcess(u32Ticks);
}

/**
  * @brief OTA package process
  * @param      None
  * @return     None
  * @details    OTA package process
  */
void OTA_WiFiProcess(void)
{
    OTA_API_WiFiProcess();
}

/**
  * @brief SD Host interrupt process
  * @param      None
  * @return     None
  * @details    SD Host interrupt process
  */
//#if (OTA_UPGRADE_FROM_SD)
void OTA_SDH_Process(void)
{
    SDH_Process();
}
//#endif


/**
  * @brief Force to execute OTA update
  * @param      None
  * @retval     0                      Success
  * @retval     others                 Failed
  * @details    Force to execute OTA update
  */
int32_t OTA_ForceUpdate(void)
{
    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
    /* Update OTA status */
#if (OTA_UPGRADE_FROM_SD)
    FMC_Write(SYS_FW_OTA_STATUS_BASE, 1);
    if (FMC_Read(SYS_FW_OTA_STATUS_BASE) != 1)
    {
        return (-1001);
    }
#else
    FMC_Write(OTA_STATUS_BASE, 1);
    if (FMC_Read(OTA_STATUS_BASE) != 1)
    {
        return (-1003);
    }
#endif
    /* Disconnect remote wifi connection */
    OTA_API_TransferConnClose();
    /* Reset CPU for update firmware */
    OTA_API_SetResetFlag();

    return 0;
}

/**
  * @brief      Get NuBL32 F/W Version
  * @param[in]  * pu32FwVer F/W version write buffer  \n
  * @param[in]  u8Mode      F/W version of NuBL32. bit-0: 0: NuBL32  \n
  * @retval     0           Success
  * @retval     others      Failed
  * @details    This function is used to get F/W version of NuBL32. \n
  *             Flow: \n
  *                 1. Get NuBL3x info \n
  *                 2. Get NuBL3x F/W version (enclosed in F/W info) \n
  */
int32_t OTA_GetBLxFwVer(uint32_t * pu32FwVer, uint8_t u8Mode)
{
    volatile int32_t    i, ret = -1000;
    uint32_t            *infobuf, base, len;
    FW_INFO_T           FwInfo;
    int32_t             method;

    if(!(u8Mode == 0))
    {
        NUBL_MSG("\nGet NuBL3x Version FAIL. Invalid mode: 0x%x.\n\n", u8Mode);
        return ret;
    }

    NUBL_MSG("\nGet NuBL32. \n\n");

    memset(&FwInfo, 0x0, sizeof(FW_INFO_T));

    infobuf = (uint32_t *)&FwInfo;

/* Step 1. Get NuBL3x info */
    /* Get NuBL3x F/W info */
    len = sizeof(FW_INFO_T);
    if((u8Mode&BIT4) != BIT4)
    {
        FMC_Open();
        if((u8Mode&BIT0) == 0)
            base = NUBL32_FW_INFO_BASE;   // encrypted NuBL32 info address

        for(i=0; i<(len/4); i++)
            infobuf[i] = FMC_Read(base + (i*4));
    }
    NUBL_MSG("Get NuBL32 F/W info [Done]\n\n");

/* Step 2. Get NuBL3x F/W version (enclosed in F/W info) */
    memcpy(&FwInfo, infobuf, sizeof(FW_INFO_T));
    memcpy(pu32FwVer, &FwInfo.mData.au32ExtInfo[0], sizeof(uint32_t));

    NUBL_MSG(" NuBL32 F/W version is [0x%08x]\n\n", FwInfo.mData.au32ExtInfo[0]);

    ret = 0;

_exit_OTA_GetBLxFwVer:
    SYS_ResetModule(CRPT_RST);

    memset(&FwInfo, 0x0, sizeof(FW_INFO_T));

    return ret;
}

/**
  * @brief      Perform AES-256 CFB NoPadding decrypt
  */
int32_t cmd_AES256Decrypt(uint32_t *in, uint32_t *out, uint32_t len, uint32_t *KEY, uint32_t *IV)
{
    uint32_t u32TimeOutCnt;

    CLK->AHBCLK |= CLK_AHBCLK_CRPTCKEN_Msk;

    /* KEY and IV are byte order (32 bit) reversed, Swap32(x)) and stored in ISP_INFO_T */
    memcpy((void *)&CRPT->AES0_KEY[0], KEY, (4 * 8));
    memcpy((void *)&CRPT->AES0_IV[0], IV, (4 * 4));

    CRPT->AES0_SADDR = (uint32_t)in;
    CRPT->AES0_DADDR = (uint32_t)out;
    CRPT->AES0_CNT   = len;
    CRPT->AES_CTL = ((AES_KEY_SIZE_256 << CRPT_AES_CTL_KEYSZ_Pos) | (AES_IN_OUT_SWAP << CRPT_AES_CTL_OUTSWAP_Pos));
    CRPT->AES_CTL |= ((AES_MODE_CFB << CRPT_AES_CTL_OPMODE_Pos) | CRPT_AES_CTL_START_Msk | CRPT_AES_CTL_DMAEN_Msk);
//    CRPT->AES_CTL |= ((AES_MODE_ECB << CRPT_AES_CTL_OPMODE_Pos) | CRPT_AES_CTL_START_Msk | CRPT_AES_CTL_DMAEN_Msk);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(CRPT->AES_STS & CRPT_AES_STS_BUSY_Msk)
    {
        if( --u32TimeOutCnt == 0 )
            return -1;
    }

    return 0;
}

/**
  * @brief      Checks before firmware upgrade
  * @param[in]  * pu32FwVer F/W version write buffer  \n
  * @param[in]  u8Mode      F/W version of NuBL32. bit-0: 0: NuBL32  \n
  * @retval     0           Success
  * @retval     others      Failed
  * @details    Check flash space, update OTA status, update FW INFO(update on-the-fly) before upgrade
  */
int32_t FwUpgradePreCheck(ISP_INFO_T *pISPInfo, CMD_PACKET_T *pCmd, FW_INFO_T *pFwInfo, int32_t i32Mode)
{
    uint32_t u32FwInfoBase;

#if (OTA_UPGRADE_FROM_SD)
    /* Can pre-check free space of SD card before write NuBL3x firmware package. */

    /* New a NuBL3x Firmware package file. */
    if (OTA_API_SDFwPackWriteOpen(g_u8NuBL3xAuthSel) == 0)
    {
        /* Write FW INFO to file. */
        if (OTA_API_SDWrite((uint8_t *)&g_RecvFwInfo, sizeof(g_RecvFwInfo)) != 0)
        {
            /* Write FW INFO to file has error. close file */
            OTA_API_SDClose(g_u8NuBL3xAuthSel);

            /* Generate response command and send it out */
            OTA_GenRspPacket(pCmd, pCmd->u16PacketID, pISPInfo, ERR_ISP_WRITE);
            return (-1);
        }
    }
    else
    {
        /* NuBL3x FW package open error */
        OTA_API_SDClose(g_u8NuBL3xAuthSel);
        /* Generate response command and send it out */
        OTA_GenRspPacket(pCmd, pCmd->u16PacketID, pISPInfo, ERR_ISP_WRITE);
        return (-1);
    }
    g_u32LastSysFwWriteAddr = pFwInfo->mData.au32FwRegion[0].u32Start;

#else /* Update on the fly */

    /* Check NuBL3x flash size is enough to write. */
    if (CheckNuBL3xWriteSpace(pFwInfo->mData.au32FwRegion[0].u32Size,i32Mode) != 0)
    {
        /* NuBL3x flash size is not enough for new firmware */
        /* Generate response command and send it out */
        OTA_GenRspPacket(pCmd, pCmd->u16PacketID, pISPInfo, ERR_ISP_WRITE);
        return (-1);
    }

    /* Re-boot for update firmware by NuBL2 */
    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    if ((FMC_Read(OTA_STATUS_BASE) != 1))
    {
        FMC_Erase(OTA_STATUS_BASE);
        FMC_Write(OTA_STATUS_BASE, 0x1UL);
        /* Disconnect local wifi connection */
        OTA_API_TransferConnClose();
        /* Disconnect remote wifi connection */
        /* Generate response command and send it out */
        OTA_GenRspPacket(pCmd, pCmd->u16PacketID, pISPInfo, STS_REBOOT);

//        if ((FMC_Read(OTA_STATUS_BASE) == 1))
        {
            //printf("update OTA status done. rebooting...\n");
            //while(!(UART_IS_TX_EMPTY(UART3))){}
            OTA_API_SetResetFlag();

            //return 0;
            return 1;
        }
//        else
//        {
//            printf("[ERR]MassWriteReqProcess: write OTA status error\n");
//            return;
//        }
    }

    if (i32Mode == 0)
        u32FwInfoBase = NUBL32_FW_INFO_BASE;

    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
    /* Update NuBL3x FW INFO */
    if (NuBL2_UpdateNuBL3xFwInfo((uint32_t *)pFwInfo, sizeof(FW_INFO_T), i32Mode, u32FwInfoBase) != 0)
    {
        printf("NuBL32 FW INFO write error\n");
        /* Generate response command and send it out */
        OTA_GenRspPacket(pCmd, pCmd->u16PacketID, pISPInfo, ERR_ISP_WRITE);
    }
#endif
    return 0;
}

/**
  * @brief      CMD_MASS_WRITE command process
  * @param[in]  pISPInfo    The pointer of ISP Info
  * @retval     0           Success
  * @retval     others      Failed
  * @details    CMD_MASS_WRITE command process
  */
int32_t MassWriteReqProcess(ISP_INFO_T *pISPInfo)
{
    uint32_t u32Fw1Size, u32FwInfoSize = 0, u32StartAddr, u32CurAddr, u32Ret = 0, u32TotalFwSize, u32Fw1Start;
    int32_t i32Mode = 0;
    uint32_t * pNuBL3xKey;
    CMD_PACKET_T cmd;
    uint32_t u32RecvPackageSize;

    memset(&cmd, 0x0, sizeof(CMD_PACKET_T));
    memcpy(&cmd, pISPInfo->rcvbuf, sizeof(cmd));

    DEBUG_MSG("MassWriteReqProcess(PID:%d)\n",cmd.u16PacketID);

    u32RecvPackageSize =(cmd.u16PacketID * 48); // maximum data length is 48

    SYS_UnlockReg();
    FMC_Open();
    /* Config update information by Firmware identity result */
    if (g_u8NuBL3xAuthSel == BIT0)
    {
        /* NuBL32 */
        i32Mode = 0;
        pNuBL3xKey = (uint32_t *)&gNuBL2_32Key;
    }

    /* Verify received FW_INFO from server */
    if (g_u32NuBL3xIdentifyPass == 0)
    {
        /* Check if FW_INFO has received finished, or copy to buffer. */
        if (u32RecvPackageSize >= FW_INFO_SIZE)
        {
            printf("MassWriteReqProcess: FW INFO verify\n");
            memcpy((uint8_t *)&g_RecvFwInfo, g_au32WriteFwBuf, sizeof(g_RecvFwInfo));
            memcpy((uint8_t *)&g_DecryptRecvFwInfo, g_au32WriteFwBuf, sizeof(g_RecvFwInfo));

            /* Clear g_u32FwInfoWriteBytes */
            g_u32FwInfoWriteBytes = 0;
            /* Verify NuBL3x identity */
            if (NuBL2_ExecuteVerifyNuBL3x((uint32_t *)&g_DecryptRecvFwInfo, 0x10|i32Mode) != 0)
            {
                printf("MassWriteReqProcess: identify error\n");
                /* Generate response command and send it out */
                OTA_GenRspPacket(&cmd, cmd.u16PacketID, pISPInfo, ERR_CMD_IDENTIFY);
            }
            else
            {
                uint32_t u32FwInfoBase;
                int32_t i32Ret;

                printf("MassWriteReqProcess: NuBL32 identify pass\n");
                g_u32NuBL3xIdentifyPass = 1;

                /* Check firmware version in FW INFO. */
                /* TODO: need verify local FW INFO hash first. */
                if (NuBL2_CompareNuBL3xVer(g_RecvFwInfo.mData.au32ExtInfo[0], i32Mode) != 0)
                {
                    printf("MassWriteReqProcess: remote NuBL32 Firmware is old in FW INFO.\n");
                    /* Generate response command and send it out */
                    OTA_GenRspPacket(&cmd, cmd.u16PacketID, pISPInfo, ERR_OLD_FW_VER);
                #if (OTA_UPGRADE_FROM_SD)
                    goto _CheckOTAStatus;
                #else
                    printf("g_u8NuBL3xAuthSel:0x%x\n",g_u8NuBL3xAuthSel);
                    if (g_u8NuBL3xAuthSel == BIT0)
                    {
//                        SYS_UnlockReg();
//                        FMC_Open();
//                        FMC_ENABLE_AP_UPDATE();
//                        FMC_Erase(OTA_STATUS_BASE);
//                    printf("g_u8IsNuBL3xFwDone: %d\n",g_u8IsNuBL3xFwDone);
//                    if (g_u8IsNuBL3xFwDone)
//                        goto _UpdateDone;
                        SYS_UnlockReg();
                        FMC_Open();
                        //check current firmware
                        printf("VECMAP = 0x%x\n", FMC_GetVECMAP());
                        if (FMC_GetVECMAP() == NUBL2_FW_BASE)
                        {
                            /* disconnect local wifi connection */
                            OTA_API_TransferConnClose();
                            printf("g_u8IsFwUpgradeDone\n");
                            g_u8IsFwUpgradeDone = TRUE;
                        }
                        else
                        {
                            goto _DisConn;
                        }
                    }
                        return 0;
                #endif
                }
                /* Check flash space and update OTA status before firmware upgrade */
                i32Ret = FwUpgradePreCheck(pISPInfo, &cmd, &g_RecvFwInfo, i32Mode);
                if (i32Ret != 0)
                {
                    if (i32Ret == 1)
                    {
                        printf("MassWriteReqProcess: re-boot for entry update mode\n");
                        return 0;
                    }
                    else
                    {
                        printf("MassWriteReqProcess: FwUpgradePreCheck error\n");
                        return (-1);
                    }
                }

            #if (!(OTA_UPGRADE_FROM_SD))
                g_u32LastSysFwWriteAddr = g_RecvFwInfo.mData.au32FwRegion[0].u32Start;
                if(i32Mode == 0)
                {
                    /* Erase old NuBL32 firmware */
                    EraseNewSysFwBlock();
                }
            #endif
            }
        }
        else /* keep encrypt FW_INFO to g_au32FwInfoBuf[160] */
        {
            memcpy((uint8_t *)&g_au32WriteFwBuf[g_u32FwInfoWriteBytes/4], (uint8_t *)cmd.au32Data, cmd.u16Len);
            g_u32FwInfoWriteBytes += (cmd.u16Len);

            /* Send write response to host */
            OTA_GenRspPacket(&cmd, cmd.u16PacketID, pISPInfo, STS_OK);

            return 0;
        }
    }

    u32TotalFwSize = g_RecvFwInfo.mData.au32FwRegion[0].u32Size + g_RecvFwInfo.mData.au32FwRegion[1].u32Size;;
    u32Fw1Size = g_RecvFwInfo.mData.au32FwRegion[0].u32Size;
    u32Fw1Start = g_RecvFwInfo.mData.au32FwRegion[0].u32Start;
    memcpy((uint8_t *)&g_au32RawData, cmd.au32Data, cmd.u16Len);

    memcpy((uint8_t *)&g_au32DecryptRawData + 16, cmd.au32Data, cmd.u16Len);

#if (OTA_UPGRADE_FROM_SD)
    /* Write new firmware package file to SD card */
    if (OTA_API_SDWrite((uint8_t *)&g_au32RawData, sizeof(g_au32RawData)) != 0)
    {
        /* Close firmware package file */
        OTA_API_SDClose(g_u8NuBL3xAuthSel);
        /* Generate response command and send it out */
        OTA_GenRspPacket(&cmd, cmd.u16PacketID, pISPInfo, ERR_ISP_WRITE);
        return (-1);
    }
    g_u32LastSysFwWriteAddr += sizeof(g_au32RawData);
#else /* Update on the fly */
    if (cmd.u16PacketID == 7)
    {
        /* First raw data package */
        if( cmd_AES256Decrypt(g_au32RawData, g_au32RawData, sizeof(g_au32RawData), pNuBL3xKey, g_FwIV) != 0 )
        {
            return (-1);
        }
        /* CFB mode need bofore 16 bytes to be decrypted current raw data correctly. */
        memcpy((uint8_t *)&g_au32DecryptRawData, &cmd.au32Data[8], 16);
    }
    else
    {
        /* Not first raw data package, so CFB mode need bofore 16 bytes to be decrypted current raw data correctly. */
        if( cmd_AES256Decrypt(g_au32DecryptRawData, g_au32DecryptRawData, sizeof(g_au32DecryptRawData), pNuBL3xKey, g_FwIV) != 0 )
        {
            return (-1);
        }
        memcpy((uint8_t *)&g_au32RawData, (uint8_t *)&g_au32DecryptRawData + 16, cmd.u16Len);
        /* CFB mode need bofore 16 bytes to be decrypted current raw data correctly. */
        memcpy((uint8_t *)&g_au32DecryptRawData, &cmd.au32Data[8], 16);
    }

    /* Check last write address is over FW1 start + FW1 size */
    if(((g_u32LastSysFwWriteAddr) + cmd.u16Len) > u32Fw1Start + u32Fw1Size)
    {
        uint32_t u32OverFwSize;

        u32OverFwSize = (g_u32LastSysFwWriteAddr) + cmd.u16Len - (u32Fw1Start + u32Fw1Size);
        /* Check if last write address is more the payload size of one package than FW1 range */
        if (u32OverFwSize < sizeof(g_au32RawData))
        {
            /* Write new firmware data to flash */
            if (OTA_WriteNewFW(g_u32LastSysFwWriteAddr, (uint8_t *)&g_au32RawData, cmd.u16Len - u32OverFwSize) != STATUS_SUCCESS)
            {
                printf("MassWriteReqProcess: write new system firmware to flash error\n");
                /* Generate response command and send it out */
                OTA_GenRspPacket(&cmd, cmd.u16PacketID, pISPInfo, ERR_ISP_WRITE);

                return (-1);
            }

            if (g_RecvFwInfo.mData.au32FwRegion[1].u32Size != 0)
            {
                /* Check write address of FW INFO is valid in system */
                if((u32Ret =_IsValidFlashRegion(g_RecvFwInfo.mData.au32FwRegion[1].u32Start, g_RecvFwInfo.mData.au32FwRegion[1].u32Size)) != 0)
                    return u32Ret;

                g_u32LastSysFwWriteAddr = g_RecvFwInfo.mData.au32FwRegion[1].u32Start;
                /* Write new firmware data to flash */
                if (OTA_WriteNewFW(g_u32LastSysFwWriteAddr, (uint8_t *)&g_au32RawData + (cmd.u16Len - u32OverFwSize), u32OverFwSize) != STATUS_SUCCESS)
                {
                    printf("MassWriteReqProcess: write new system firmware to flash error\n");
                    /* Generate response command and send it out */
                    OTA_GenRspPacket(&cmd, cmd.u16PacketID, pISPInfo, ERR_ISP_WRITE);

                    return (-1);
                }
            }
        }
        else
        {
            if (u32OverFwSize == sizeof(g_au32RawData))
            {
                if (g_RecvFwInfo.mData.au32FwRegion[1].u32Size != 0)
                {
                    /* Check write address of FW INFO is valid in system */
                    if((u32Ret =_IsValidFlashRegion(g_RecvFwInfo.mData.au32FwRegion[1].u32Start, g_RecvFwInfo.mData.au32FwRegion[1].u32Size)) != 0)
                        return u32Ret;

                    g_u32LastSysFwWriteAddr = g_RecvFwInfo.mData.au32FwRegion[1].u32Start;
                }
            }
            /* Write new firmware data to flash */
            if (OTA_WriteNewFW(g_u32LastSysFwWriteAddr, (uint8_t *)&g_au32RawData, cmd.u16Len) != STATUS_SUCCESS)
            {
                printf("MassWriteReqProcess: write new system firmware to flash error\n");
                /* Generate response command and send it out */
                OTA_GenRspPacket(&cmd, cmd.u16PacketID, pISPInfo, ERR_ISP_WRITE);

                return (-1);
            }
        }
    }
    else
    {
        /* Write new firmware data to flash */
        if (OTA_WriteNewFW(g_u32LastSysFwWriteAddr, (uint8_t *)&g_au32RawData, cmd.u16Len) != STATUS_SUCCESS)
        {
            printf("MassWriteReqProcess: write new system firmware to flash error\n");
            /* Generate response command and send it out */
            OTA_GenRspPacket(&cmd, cmd.u16PacketID, pISPInfo, ERR_ISP_WRITE);

            return (-1);
        }
    }
#endif

    if ((cmd.u16Len) < MAX_PAYLOAD_SIZE)
    {
        /* Case: update on the fly method for only NuBL32 update done, NuBL32 need not to be updated. */
        g_u8IsNuBL3xFwDone = 1;
    #if (OTA_UPGRADE_FROM_SD)
        /* Close firmware package file */
        OTA_API_SDClose(g_u8NuBL3xAuthSel);

        SYS_UnlockReg();
        FMC_Open();
        FMC_ENABLE_AP_UPDATE();
        /* Update OTA status */
        if (g_u8NuBL3xAuthSel&BIT0)
        {
            FMC_Write(SYS_FW_OTA_STATUS_BASE, 1);
        }
    #endif
    }

    /* Generate response command and send it out */
    OTA_GenRspPacket(&cmd, cmd.u16PacketID, pISPInfo, STS_OK);

    return 0;

_CheckOTAStatus:
    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();
#if (OTA_UPGRADE_FROM_SD)
    /* Check OTA status */
    if (FMC_Read(SYS_FW_OTA_STATUS_BASE) == 1)
        goto _UpdateDone;
    else
    {
        if (g_u8NuBL3xAuthSel == BIT1)
        {
            goto _DisConn;
        }
        else
            return 0;
    }
#else
    if (FMC_Read(OTA_STATUS_BASE) != 1)
        return 0;
#endif

_UpdateDone:
    /* NuBL32 has update done. */

    /* Disconnect local wifi connection */
    OTA_API_TransferConnClose();

    /* Re-identify NuBL32 for next connection */
    g_u32NuBL3xIdentifyPass = 0;
    /* Clear g_u32FwInfoWriteBytes */
    g_u32FwInfoWriteBytes = 0;

    OTA_API_SetResetFlag();

_DisConn:
    /* NuBL32 has update done. */

    /* Disconnect local wifi connection */
    OTA_API_TransferConnClose();

    /* Re-identify NuBL32 for next connection */
    g_u32NuBL3xIdentifyPass = 0;
    /* Clear g_u32FwInfoWriteBytes */
    g_u32FwInfoWriteBytes = 0;

    return u32Ret;
}

/**
  * @brief      CMD_DISCONNECT command process
  * @param      None
  * @retval     0           Success
  * @retval     others      Failed
  * @details    CMD_DISCONNECT command process
  */
int32_t DisConnectReqProcess(void)
{
    uint32_t u32IsUpgradeDone;
    int32_t i32Ret;

    u32IsUpgradeDone = g_u8SetFwUpgradeDone;

    /* Disconnect local wifi connection */
    OTA_API_TransferConnClose();

    /* Re-identify NuBL32 for next connection */
    g_u32NuBL3xIdentifyPass = 0;
    /* Clear g_u32FwInfoWriteBytes */
    g_u32FwInfoWriteBytes = 0;

    /* This global structure need to be initialized before re-connect, because client and server key was changed to random public key. */
    i32Ret = IspInfo_Init(g_pISPInfo);

    if (u32IsUpgradeDone)
    {
        /* Set Reset flag for transfer task */
        OTA_API_SetResetFlag();
    }

#if (!(OTA_UPGRADE_FROM_SD))
    if (u32IsUpgradeDone)
    {
        printf("g_u8IsFwUpgradeDone\n");
        g_u8IsFwUpgradeDone = TRUE;
    }
#endif
    return i32Ret;
}

/**
  * @brief      OTA commands process
  * @param[in]  pISPInfo    The pointer of ISP Info
  * @retval     0           Success
  * @retval     others      Failed
  * @details    OTA commands process
  */
int32_t OTACmdReqProcess(ISP_INFO_T *pISPInfo)
{
    volatile int32_t    i, u32Ret = 0;
    CMD_PACKET_T        cmd;

    memset(&cmd,             0x0, sizeof(CMD_PACKET_T));
    memset(pISPInfo->rspbuf, 0x0, sizeof(pISPInfo->rspbuf));
    /* Copy received content of command from receive buffer of ISP Info  */
    memcpy(&cmd, pISPInfo->rcvbuf, sizeof(cmd));

    /* Parse received command packet, including decrypted data. */
    if(Cmd_ParseReqPacket(&cmd, pISPInfo) != 0)
    {
        DEBUG_MSG("*** [Parse error: 0x%x] ***\n", cmd.u16CmdID);
        memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));

        /* Response checksum error */
        cmd.au32Data[0] = ERR_CMD_CHECKSUM;
        cmd.u16Len      = (4 * 1);
        u32Ret = -1;
    }
    else
    {
        switch(cmd.u16CmdID)
        {
            case CMD_ECDH_PUB0:
                /* Get Server public key 0 */
                memcpy(pISPInfo->ServerPubKey.au32Key0, cmd.au32Data, cmd.u16Len);
                for(i=0; i<8; i++)
                    NUBL_MSG("Get pub0[%d]: 0x%08x.\n", i, pISPInfo->ServerPubKey.au32Key0[i]);

                /* Response status ok */
                memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                cmd.au32Data[0] = STS_OK;
                cmd.u16Len      = (4 * 1);
                u32Ret = cmd.u16CmdID;
            break;
            case CMD_ECDH_PUB1:
                /* Get Server public key 1 and generate ist ECDH AES key */
                memcpy(pISPInfo->ServerPubKey.au32Key1, cmd.au32Data, cmd.u16Len);
                for(i=0; i<8; i++)
                    NUBL_MSG("Get pub1[%d]: 0x%08x.\n", i, pISPInfo->ServerPubKey.au32Key1[i]);

                /* Identify Host public key */
                if((IdentifyPublicKey((uint32_t *)pISPInfo->ServerPubKey.au32Key0, 0)&BIT2) != BIT2)
                {
                    /* Response key authentication error */
                    cmd.au32Data[0] = ERR_AUTH_KEY;
                    cmd.u16Len      = (4 * 1);
                    u32Ret = -1;
                    break;
                }
                /* Response status ok */
                memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                cmd.au32Data[0] = STS_OK;
                cmd.u16Len      = (4 * 1);
                u32Ret = cmd.u16CmdID;
            break;
            case CMD_AUTH_KEY:
                /* compare NuBL3x pubKey from host */
                g_u8NuBL3xAuthSel = VerifyNuBL3xKeyHash(((CMD_AUTH_KEY_REQ_T *)&cmd)->au32KeyHash);
                printf("g_u8NuBL3xAuthSel: 0x%x\n", g_u8NuBL3xAuthSel);
                memset(cmd.au32Data, 0x0, sizeof(cmd.au32Data));
                if ((g_u8NuBL3xAuthSel == BIT0)||(g_u8NuBL3xAuthSel == BIT1)||(g_u8NuBL3xAuthSel == (BIT0|BIT1)))
                {
                    /* verify pass */
                    cmd.au32Data[0] = STS_OK;
                    cmd.u16Len      = (4 * 1);
                    u32Ret = cmd.u16CmdID;
                }
                else
                {
                    /* verify failed */
                    cmd.au32Data[0] = ERR_AUTH_KEY;
                    cmd.u16Len      = (4 * 1);
                    u32Ret = -1;
                }
                /* clear g_u32NuBL3xIdentifyPass for re-verify FW INFO */
                g_u32NuBL3xIdentifyPass = 0;
                /* clear g_u32FwInfoWriteBytes */
                g_u32FwInfoWriteBytes = 0;
            break;
            case CMD_MASS_WRITE:
                memcpy(pISPInfo->rcvbuf, &cmd, sizeof(cmd));
                MassWriteReqProcess(pISPInfo);
                return 0;
            break;
            case CMD_DISCONNECT:
                /* get F/W upgrade done inform and keep to global structure. */
                g_u8SetFwUpgradeDone = cmd.au32Data[0];

                /* Response status ok */
                cmd.au32Data[0] = STS_OK;
                cmd.u16Len      = (4 * 1);
                u32Ret = cmd.u16CmdID;
            break;

            default:
                printf("Invalid command: 0x%x\n", cmd.u16CmdID);
            break;
        }
    }
    if(u32Ret == -1)
        pISPInfo->IsConnectOK = 0;

    /* Generate response command packet */
    Cmd_GenRspPacket(&cmd, pISPInfo);

    /* Copy response command to response buffer of ISP Info */
    memcpy(pISPInfo->rspbuf, (uint8_t *)&cmd, sizeof(CMD_PACKET_T));

    return u32Ret;
}

/**
  * @brief      OTA commands handler
  * @param[in]  pu8Buff        The pointer of packet buffer
  * @param[in]  u32Len         Valind data length
  * @param[in]  u32StartIdx    Valind data index in packet buffer
  * @param[in]  u32ValidLen    Valind data length
  * @return     None
  * @details    OTA commands handler
  */
void OTA_CallBackHandler(uint8_t* pu8Buff, uint32_t u32Len, uint32_t u32StartIdx, uint32_t u32ValidLen)
{
    uint16_t u16CmdID;
    volatile int32_t i32Ret = 0;

    /* Copy received packet to receive buffer of ISP Info */
    memcpy(g_pISPInfo->rcvbuf, (uint32_t *)pu8Buff, u32Len);

    DEBUG_MSG("OTA_CallBackHandler(0x%x)\n",((CMD_PACKET_T *)g_pISPInfo->rcvbuf)->u16CmdID);

    switch(((CMD_PACKET_T *)g_pISPInfo->rcvbuf)->u16CmdID)
    {
        //NuBL1 libary commands process ---------------------------------------------------------------
        case CMD_CONNECT:
            DEBUG_MSG("CMD_CONNECT_REQ\n");
            i32Ret = ParseCONNECT(g_pISPInfo);
            /* Prepare NuBL2_32 ECDH key for F/W decrypt */
            NuBL2_GetNuBL3xECDHKeys((uint32_t *)gNuBL2_32Key, NULL);

            g_u32NuBL3xIdentifyPass = 0;
            /* Send command packet */
            OTA_API_SendFrame((uint8_t *) g_pISPInfo->rspbuf, MAX_FRAME_SIZE);
        break;

        case CMD_ECDH_GET_PUB0:
        case CMD_ECDH_GET_PUB1:
        case CMD_ECDH_RAND_PUB0:
        case CMD_ECDH_RAND_PUB1:
        case CMD_ECDH_GET_RAND_PUB0:
        case CMD_ECDH_GET_RAND_PUB1:
            i32Ret = ParseECDH(g_pISPInfo);
            /* Send command packet */
            OTA_API_SendFrame((uint8_t *) g_pISPInfo->rspbuf, MAX_FRAME_SIZE);
        break;
        case CMD_SET_MASS_WRITE:
        case CMD_RESET: //need ??(CHIP reset?System reset?CPU reset)
            i32Ret = ParseCommands(g_pISPInfo);
            /* Send command packet */
            OTA_API_SendFrame((uint8_t *) g_pISPInfo->rspbuf, MAX_FRAME_SIZE);
        break;

        //OTA customized process ----------------------------------------------------------------------
        case CMD_ECDH_PUB0:
        {
            uint32_t AESKey[8], i;
            /* Generate 1st ECDH AES key */
            /* Calculate (NuBL2 priv * Host pub) ECDH key. because NuBL2 has known the public key of host. */
            if(GenCmdSessionKey(AESKey) != 0)
            {
                /* Clear key buffer */
                memset(&AESKey, 0x0, sizeof(AESKey));
                return ;
            }
            for(i=0; i<8; i++)
                AESKey[i] = Swap32(AESKey[i]);
            NuBL_BytesSwap((char *)&AESKey, sizeof(AESKey));

            /* Copy session key to ISP Info */
            memcpy((void *)g_pISPInfo->au32AESKey, AESKey, sizeof(AESKey));
            for(i=0; i<8; i++)
                NUBL_MSG("Gen 1st KEY[%d]: 0x%08x.\n", i, g_pISPInfo->au32AESKey[i]);
        }
        case CMD_ECDH_PUB1:
        case CMD_AUTH_KEY:
        case CMD_MASS_WRITE:
        case CMD_DISCONNECT:
            /* OTA commands process */
            OTACmdReqProcess(g_pISPInfo);
            /* Send command packet */
            OTA_API_SendFrame((uint8_t *) g_pISPInfo->rspbuf, MAX_FRAME_SIZE);

            /* Disconnect Wifi connection */
            if (((CMD_PACKET_T *)g_pISPInfo->rcvbuf)->u16CmdID == CMD_DISCONNECT)
                DisConnectReqProcess();
        break;

        default:
            printf("Invalid command: 0x%x\n", ((CMD_PACKET_T *)g_pISPInfo->rcvbuf)->u16CmdID);
        break;
    }
}


/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
