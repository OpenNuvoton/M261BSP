/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief    Show a Master how to access Slave.
 *           This sample code needs to work with USCI_I2C_Slave.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK       64000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8MstTxData[3];
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstEndFlag = 0;
volatile uint8_t g_u8MstDataLen;

void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF2MFP_Msk)) | SYS_GPF_MFPL_PF2MFP_XT1_OUT;
    SYS->GPF_MFPL = (SYS->GPF_MFPL & (~SYS_GPF_MFPL_PF3MFP_Msk)) | SYS_GPF_MFPL_PF3MFP_XT1_IN;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HXT `and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Enable UI2C0 peripheral clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set PA multi-function pins for UI2C0_SDA(PA.10) and UI2C0_SCL(PA.11) */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA10MFP_Msk | SYS_GPA_MFPH_PA11MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA10MFP_USCI0_DAT0 | SYS_GPA_MFPH_PA11MFP_USCI0_CLK);

    /* I2C pins enable schmitt trigger */
    PA->SMTEN |= (GPIO_SMTEN_SMTEN10_Msk | GPIO_SMTEN_SMTEN11_Msk);
}

void UI2C0_Init(uint32_t u32ClkSpeed)
{
    /* Open USCI_I2C0 and set clock to 100k */
    UI2C_Open(UI2C0, u32ClkSpeed);

    /* Get USCI_I2C0 Bus Clock */
    printf("UI2C0 clock %d Hz\n", UI2C_GetBusClockFreq(UI2C0));

    /* Set USCI_I2C0 Slave Addresses */
    UI2C_SetSlaveAddr(UI2C0, 0, 0x15, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    UI2C_SetSlaveAddr(UI2C0, 1, 0x35, UI2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */

    /* Set USCI_I2C0 Slave Addresses Mask */
    UI2C_SetSlaveAddrMask(UI2C0, 0, 0x01);                    /* Slave Address Mask: 0x1 */
    UI2C_SetSlaveAddrMask(UI2C0, 1, 0x04);                    /* Slave Address Mask: 0x4 */
}

int main(void)
{
    uint32_t u32i;
    uint8_t u8Data, u8Tmp, u8Err;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code sets USCI_I2C bus clock to 100kHz. Then, Master accesses Slave with single byte write
        and single byte read operations, and check if the read data is equal to the programmed data.
    */

    printf("+---------------------------------------------------------+\n");
    printf("|  USCI_I2C Driver Sample Code for Master access          |\n");
    printf("|  7-bit address Slave. Needs to work with USCI_I2C_Slave |\n");
    printf("|  sample code                                            |\n");
    printf("|                                                         |\n");
    printf("|  UI2C0(Master)  <----> UI2C0(Slave)                     |\n");
    printf("+---------------------------------------------------------+\n");

    printf("\n");
    printf("Configure UI2C0 as a Master\n");
    printf("The I/O connection for UI2C0:\n");
    printf("UI2C0_SDA(PA.10), UI2C0_SCL(PA.11)\n");

    /* Init UI2C0 bus bard rate */
    UI2C0_Init(100000);

    /* Slave address */
    g_u8DeviceAddr = 0x15;

    u8Err = 0;

    for(u32i = 0; u32i < 256; u32i++)
    {
        u8Tmp = (uint8_t)u32i + 3;

        /* Single Byte Write (Two Registers) */
        while(UI2C_WriteByteTwoRegs(UI2C0, g_u8DeviceAddr, u32i, u8Tmp));

        /* Single Byte Read (Two Registers) */
        u8Data = UI2C_ReadByteTwoRegs(UI2C0, g_u8DeviceAddr, u32i);
        if(u8Data != u8Tmp)
        {
            u8Err = 1;
            printf("%03d: Single byte write data fail,  W(0x%X)/R(0x%X) \n", u32i, u8Tmp, u8Data);
        }
    }

    printf("\n");

    if(u8Err)
        printf("Single byte Read/Write access Fail.....\n");
    else
        printf("Single byte Read/Write access Pass.....\n");

    while(1);
}
/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/


