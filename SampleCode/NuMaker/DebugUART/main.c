/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A simple demo for NuTiny-M261 board to show message from UART5 to ICE VCOM.
 *
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M261.h"

void SYS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) == 0)
        if(--u32TimeOutCnt == 0) break;

    /* Set HCLK divider to 2 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(2);

    /* Switch HCLK clock source to PLL */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Select IP clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART0SEL_HIRC;

    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;


    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = 128000000;            // PLL
    SystemCoreClock = 128000000 / 2;        // HCLK
    CyclesPerUs     =  64000000 / 1000000;  // For CLK_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART0->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

}



int main()
{
    int32_t i;

    SYS_UnlockReg();

    SYS_Init();

    UART0_Init();

    /*
        In NuMaker board, the NuLink ICE also provides VCOM debug port for debug message.
        Once plug the ICE to PC, there will also be a USB VCOM on PC.
        All message print to UART0 will output to the VCOM.

        NOTE:
            User must use "Hardware Manager" of windows to identify which COM number is used for VCOM.
            Then open it by any serial terminal tool for monitor debug message.

    */

    printf("\n");
    printf("+------------------------------------------------------------------+\n");
    printf("|              Simple Debug Message Demo                           |\n");
    printf("+------------------------------------------------------------------+\n");

    /* Init GPIO for LED toggle */
    PB->MODE = (GPIO_MODE_OUTPUT << 10 * 2);
    PB10 = 1;

    i = 0;
    while(1)
    {
        PB10 ^= 1;
        CLK_SysTickLongDelay(200000);
        PB10 ^= 1;
        CLK_SysTickLongDelay(200000);

        printf("Iter: %d\n", i++);
    }


}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
