/**************************************************************************//**
 * @file     otalib.h
 * @version  V1.00
 * @brief    OTA Lib header file
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __OTALIB_H__
#define __OTALIB_H__

#ifdef __cplusplus
extern "C"
{
#endif

/**
  * @brief OTA process initialization
  * @param[in]  u32HSI      PLL Output Clock Frequency
  * @param[in]  pISPInfo    The pointer of ISP Info
  * @retval     0           Success
  * @retval     others      Failed
  * @details    OTA process initialization
  */
int32_t OTA_Init(uint32_t u32HSI, ISP_INFO_T *pISPInfo);

/**
  * @brief OTA system tick interrupt process
  * @param[in]  u32Ticks    Tick value
  * @retval     0           Success
  * @retval     others      Failed
  * @details    OTA system tick interrupt process
  */
uint8_t OTA_SysTickProcess(uint32_t u32Ticks);

/**
  * @brief OTA package process
  * @param      None
  * @return     None
  * @details    OTA package process
  */
void OTA_WiFiProcess(void);

/**
  * @brief      OTA task routine
  * @param      None
  * @retval     0             Success
  * @retval     others        Failed
  * @details    OTA task routine
  */
int8_t OTA_TaskProcess(void);

/**
  * @brief      Get NuBL32 F/W Version
  * @param[in]  * pu32FwVer F/W version write buffer  \n
  * @param[in]  u8Mode      F/W version of NuBL32. bit-0: 0: NuBL32  \n
  * @retval     0           Success
  * @retval     others      Failed
  * @details    This function is used to get F/W version of NuBL32. \n
  *             Flow: \n
  *                 1. Decrypt or get NuBL3x info \n
  *                 2. Get NuBL3x F/W version (enclosed in F/W info) \n
  */
int32_t OTA_GetBLxFwVer(uint32_t * pu32FwVer, uint8_t u8Mode);

#if (OTA_UPGRADE_FROM_SD)
/**
  * @brief Verify and Update NuBL32/33 from SD card.
  * @param[in]  u8BLxSel    NuBL3x selection. Bit0 = 1: NuBL32, Bit1 = 1: NuBL33
  * @retval     0           Success
  * @retval     others      Failed
  * @details    NuBL3x identity, authentication and firmware integrity.
  *             Update NuBL3x FW INFO and firmware.
  */
int32_t OTA_VerifyAndUpdateNuBL3xFromSD(uint8_t u8BLxSel);

/**
  * @brief SD Host interrupt process
  * @param      None
  * @return     None
  * @details    SD Host interrupt process
  */
void OTA_SDH_Process(void);
#endif

/**
  * @brief Force to execute OTA update
  * @param      None
  * @retval     0       Success
  * @retval     others  Failed
  * @details    Force to execute OTA update
  */
int32_t OTA_ForceUpdate(void);


#ifdef __cplusplus
}
#endif

#endif /* __OTALIB_H__ */

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
