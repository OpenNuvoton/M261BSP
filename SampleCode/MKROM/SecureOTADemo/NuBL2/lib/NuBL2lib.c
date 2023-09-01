/******************************************************************************
 * @file     NuBL2lib.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/10/17 2:06p $
 * @brief    Function pointer for NuBL2 APIs.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NuMicro.h"
#include "ota_api.h"

int32_t (*OTA_Init)(uint32_t u32HSI, ISP_INFO_T *pISPInfo) = (int32_t (*)(uint32_t u32HSI, ISP_INFO_T *pISPInfo))(0x00003a4d);
uint8_t (*OTA_SysTickProcess)(uint32_t u32Ticks) = (uint8_t (*)(uint32_t u32Ticks))(0x00003ac1);
void (*OTA_WiFiProcess)(void) = (void (*)(void))(0x00003b5d);
int8_t (*OTA_TaskProcess)(void) = (int8_t (*)(void))(0x00003b55);
int32_t (*OTA_GetBLxFwVer)(uint32_t * pu32FwVer, uint8_t u8Mode) = (int32_t (*)(uint32_t * pu32FwVer, uint8_t u8Mode))(0x0000395d);
int32_t (*OTA_ForceUpdate)(void) = (int32_t (*)(void))(0x00003875);
#if (OTA_UPGRADE_FROM_SD)
void (*OTA_SDH_Process)(void) = (void (*)(void))(0x00003ab9);
#endif
        
        
    