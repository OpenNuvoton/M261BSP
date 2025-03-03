/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Configure QSPI0 as Slave 3-wire mode and demonstrate how to
 *           communicate with an off-chip SPI Master device with FIFO mode.
 *           This sample code needs to work with SPI_MasterFIFOMode sample code.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <o> GPIO Slew Rate Control
// <0=> Normal <1=> High <2=> Fast
#define SlewRateMode    0
// *** <<< end of configuration section >>> ***

#define TEST_COUNT      16

static uint32_t s_au32SourceData[TEST_COUNT];
static uint32_t s_au32DestinationData[TEST_COUNT];
static volatile uint32_t s_u32TxDataCount;
static volatile uint32_t s_u32RxDataCount;

void SYS_Init(void);
void QSPI_Init(void);
void QSPI0_IRQHandler(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Enable PLL */
    CLK->PLLCTL = CLK_PLLCTL_128MHz_HIRC;

    /* Waiting for PLL stable */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Select HCLK clock source as PLL and HCLK source divider as 2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_CLKDIV0_HCLK(2));

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_HCLK_DIV2 | CLK_PCLKDIV_APB1DIV_HCLK_DIV2);

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

    /* Select PCLK0 as the clock source of QSPI0 */
    CLK_SetModuleClock(QSPI0_MODULE, CLK_CLKSEL2_QSPI0SEL_PCLK0, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable QSPI0 peripheral clock */
    CLK_EnableModuleClock(QSPI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Setup QSPI0 multi-function pins */
    SYS->GPC_MFPL &= ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk | SYS_GPC_MFPL_PC2MFP_Msk);
    SYS->GPC_MFPL |= (SYS_GPC_MFPL_PC0MFP_QSPI0_MOSI0 | SYS_GPC_MFPL_PC1MFP_QSPI0_MISO0 | SYS_GPC_MFPL_PC2MFP_QSPI0_CLK);

#if (SlewRateMode == 0)
    /* Enable QSPI0 I/O normal slew rate */
    GPIO_SetSlewCtl(PC, BIT0 | BIT1 | BIT2, GPIO_SLEWCTL_NORMAL);
#elif (SlewRateMode == 1)
    /* Enable QSPI0 I/O high slew rate */
    GPIO_SetSlewCtl(PC, BIT0 | BIT1 | BIT2, GPIO_SLEWCTL_HIGH);
#elif (SlewRateMode == 2)
    /* Enable QSPI0 I/O fast slew rate */
    GPIO_SetSlewCtl(PC, BIT0 | BIT1 | BIT2, GPIO_SLEWCTL_FAST);
#endif

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
}

void QSPI_Init(void)
{
    /* Enable slave 3-wire mode before enabling QSPI controller */
    QSPI_ENABLE_3WIRE_MODE(QSPI0);

    /* Configure as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure QSPI0 as a low level active device. */
    QSPI0->CTL = QSPI_SLAVE | QSPI_MODE_0 | QSPI_CTL_SPIEN_Msk;
}

void QSPI0_IRQHandler(void)
{
    while(s_u32RxDataCount < TEST_COUNT)
    {
        /* Check RX EMPTY flag */
        while(QSPI_GET_RX_FIFO_EMPTY_FLAG(QSPI0) == 0)
        {
            /* Read RX FIFO */
            s_au32DestinationData[s_u32RxDataCount++] = QSPI_READ_RX(QSPI0);
        }
        /* Check TX FULL flag and TX data count */
        while((QSPI_GET_TX_FIFO_FULL_FLAG(QSPI0) == 0) && (s_u32TxDataCount < TEST_COUNT))
        {
            /* Write to TX FIFO */
            QSPI_WRITE_TX(QSPI0, s_au32SourceData[s_u32TxDataCount++]);

            /* Check the Slave TX underflow interrupt flag */
            if(QSPI_GetIntFlag(QSPI0, QSPI_TXUF_INT_MASK))
            {
                QSPI_DISABLE_3WIRE_MODE(QSPI0);
                QSPI_ClearIntFlag(QSPI0, QSPI_TXUF_INT_MASK);
                QSPI_ENABLE_3WIRE_MODE(QSPI0);
            }
        }
        if(s_u32TxDataCount >= TEST_COUNT)
            QSPI_DisableInt(QSPI0, QSPI_FIFO_TXTH_INT_MASK); /* Disable TX FIFO threshold interrupt */

        /* Check the RX FIFO time-out interrupt flag */
        if(QSPI_GetIntFlag(QSPI0, QSPI_FIFO_RXTO_INT_MASK))
        {
            /* If RX FIFO is not empty, read RX FIFO. */
            while(QSPI_GET_RX_FIFO_EMPTY_FLAG(QSPI0) == 0)
                s_au32DestinationData[s_u32RxDataCount++] = QSPI_READ_RX(QSPI0);
        }
    }
}

int main(void)
{
    uint32_t u32DataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init QSPI */
    QSPI_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------------------------+\n");
    printf("|                  QSPI0 Slave 3-Wire Mode Sample Code                  |\n");
    printf("+-----------------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure QSPI0 as a slave.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for QSPI0:\n");
    printf("    QSPI0_CLK(PC2)\n    QSPI0_MISO(PC1)\n    QSPI0_MOSI(PC0)\n\n");
    printf("QSPI controller will enable FIFO mode and transfer %d data to an off-chip master device.\n", TEST_COUNT);
    printf("In the meanwhile the QSPI controller will receive %d data from the off-chip master device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);

    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        /* Write the initial value to source buffer */
        s_au32SourceData[u32DataCount] = 0x00AA0000 + u32DataCount;
        /* Clear destination buffer */
        s_au32DestinationData[u32DataCount] = 0;
    }

    s_u32TxDataCount = 0;
    s_u32RxDataCount = 0;
    printf("Press any key if the master device configuration is ready.\n");
    getchar();
    printf("\n");

    /* Set TX FIFO threshold */
    QSPI_SetFIFO(QSPI0, 2, 2);

    /* Enable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    QSPI_EnableInt(QSPI0, QSPI_FIFO_TXTH_INT_MASK | QSPI_FIFO_RXTO_INT_MASK);
    NVIC_EnableIRQ(QSPI0_IRQn);

    /* Print the received data */
    printf("Received data:\n");
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, s_au32DestinationData[u32DataCount]);
    }

    /* Disable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    QSPI_DisableInt(QSPI0, QSPI_FIFO_TXTH_INT_MASK | QSPI_FIFO_RXTO_INT_MASK);
    NVIC_DisableIRQ(QSPI0_IRQn);

    printf("The data transfer was done.\n");

    printf("\n\nExit QSPI driver sample code.\n");

    /* Reset QSPI0 */
    QSPI_Close(QSPI0);

    while(1);
}
