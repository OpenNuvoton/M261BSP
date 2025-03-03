
/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 18/05/28 2:06p $
 * @brief
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NuDB_common.h"
#include "DrvUART.h"

#define PLL_CLOCK       64000000


/* APROM bank1 address is 1/2 APROM size.   */
#define DB_PROG_LEN                 (4 * FMC_FLASH_PAGE_SIZE)  /* background program length  */
#define CRC32_LOOP_CNT              50           /* Loop count                               */



/*
 *  Dual bank background program state
 */
enum
{
    DB_STATE_START,                              /* Start background dual bank program       */
    DB_STATE_ERASE,                              /* Executing ISP page erase                 */
    DB_STATE_PROGRAM,                            /* Executing ISP write                      */
    DB_STATE_DONE,                               /* All queued ISP operations finished. Idle */
    DB_STATE_FAIL                                /* ISP command failed or verify error       */
};

static volatile int  g_i8DbState = DB_STATE_DONE;    /* dual bank background program state       */
static volatile uint32_t  g_u32DbLength;             /* dual bank program remaining length       */
static volatile uint32_t  g_u32DbAddr;               /* dual bank program current flash address  */

volatile uint32_t  g_u32TickCnt;                     /* timer ticks - 100 ticks per second       */
uint32_t g_u32TmpData, g_u32TmpCnt;

volatile uint32_t g_u32TxIndex, g_u32RxIndex;
uint8_t u8RxData[TXBUFSIZE];
UART_T *u32TestPortTx, *u32TestPort, *u32TestPortRx;


void SysTick_Handler(void)
{
    g_u32TickCnt++;                                 /* increase timer tick                      */

    if(g_i8DbState == DB_STATE_DONE)                /* Background program is in idle state      */
    {
        return;
    }

    if(g_u32DbLength == 0)                          /* Background program done?                 */
    {
        g_i8DbState = DB_STATE_DONE;                /* enter idle state                         */
        return;
    }

    if(FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk)
        return;                                  /* ISP is busy, postpone to next called     */

    /*
     *  Dual-bank background program...
     */
    switch(g_i8DbState)
    {
        case DB_STATE_START:
            if(g_u32DbAddr & ~FMC_PAGE_ADDR_MASK)
            {
                printf("Warning - dual bank start address is not page aligned!\n");
                g_i8DbState = DB_STATE_FAIL;
                break;
            }
            if(g_u32DbLength & ~FMC_PAGE_ADDR_MASK)
            {
                printf("Warning - dual bank length is not page aligned!\n");
                g_i8DbState = DB_STATE_FAIL;
                break;
            }
            g_i8DbState = DB_STATE_ERASE;           /* Next state is to erase flash            */
            break;

        case DB_STATE_ERASE:
            FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;     /* ISP page erase command                   */
            FMC->ISPADDR = g_u32DbAddr;              /* page address                             */
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;      /* trigger ISP page erase and no wait       */
            printf("Erase [0x%8x].\n", g_u32DbAddr);
            g_u32DbAddr += TMP_PAGE_SIZE;

            if(g_u32DbAddr >= (BANK1_FW_BASE + BANK1_FW_SIZE))
            {
                printf("\nErase Done g_u32DbAddr[0x%8x]\n", g_u32DbAddr);
                g_u32DbAddr = BANK1_FW_BASE;
                g_i8DbState = DB_STATE_PROGRAM;         /* Next state is to program flash           */
                g_u32TmpCnt = 0;
            }
            break;

        case DB_STATE_PROGRAM:
            g_u32TmpCnt =  BANK1_FW_SIZE - g_u32DbLength;
            g_u32TmpData = u8RxData[g_u32TmpCnt] | (u8RxData[g_u32TmpCnt + 1] << 8) | (u8RxData[g_u32TmpCnt + 2] << 16) | (u8RxData[g_u32TmpCnt + 3] << 24);

            FMC->ISPCMD  = FMC_ISPCMD_PROGRAM;    /* ISP word program command                 */
            FMC->ISPADDR = g_u32DbAddr;               /* word program address                     */
            FMC->ISPDAT  = g_u32TmpData;               /* 32-bits data to be programmed            */
            FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;  /* trigger ISP program and no wait          */

            g_u32DbAddr += 4;                          /* advance to next word                    */
            g_u32DbLength -= 4;

            if(g_u32DbAddr % 0x800 == 0)
                printf("[0x%8x] page programing done\n", g_u32DbAddr);

            if(g_u32DbLength == 0)
                printf("\nAll programing done!!!!!g_u32DbAddr[0x%8x]\n", g_u32DbAddr);


            break;

        default:
            printf("Unknown g_i8DbState state!\n");
            break;
    }
}

void enable_sys_tick(int i8TicksPerSecond)
{
    g_u32TickCnt = 0;
    SystemCoreClock = PLL_CLOCK;         /* HCLK is 64 MHz */
    if(SysTick_Config(SystemCoreClock / i8TicksPerSecond))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
    }
}

void WDT_IRQHandler(void)
{
    WDT_RESET_COUNTER();

    if(WDT_GET_TIMEOUT_INT_FLAG() == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();
    }


}



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
    CLK_EnableModuleClock(UART1_MODULE);
    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HXT, CLK_CLKDIV0_UART1(1));


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;

    /* Set multi-function pins for UART1 RXD and TXD form downloading new FW*/
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB6MFP_Msk)) | (SYS_GPB_MFPL_PB6MFP_UART1_RXD);
    SYS->GPB_MFPL = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB7MFP_Msk)) | (SYS_GPB_MFPL_PB7MFP_UART1_TXD);
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB8MFP_Msk)) | (SYS_GPB_MFPH_PB8MFP_UART1_nRTS);
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~SYS_GPB_MFPH_PB9MFP_Msk)) | (SYS_GPB_MFPH_PB9MFP_UART1_nCTS);
    SYS->GPB_MFOS = (SYS->GPB_MFOS & (~SYS_GPB_MFOS_MFOS6_Msk)) | (0 << SYS_GPB_MFOS_MFOS6_Pos);
    SYS->GPB_MFOS = (SYS->GPB_MFOS & (~SYS_GPB_MFOS_MFOS7_Msk)) | (0 << SYS_GPB_MFOS_MFOS7_Pos);
    SYS->GPB_MFOS = (SYS->GPB_MFOS & (~SYS_GPB_MFOS_MFOS8_Msk)) | (0 << SYS_GPB_MFOS_MFOS8_Pos);
    SYS->GPB_MFOS = (SYS->GPB_MFOS & (~SYS_GPB_MFOS_MFOS9_Msk)) | (0 << SYS_GPB_MFOS_MFOS9_Pos);
}


uint32_t  SelfTest(void)
{
    uint32_t  i,  sum;
    for(i = 0; i < 1; i++)
    {
        sum = func_crc32(BANK0_FW_BASE, BANK0_FW_SIZE);
    }
    printf("sum = 0x%x", sum);
    return 1;
}

void start_timer0()
{
    /* Start TIMER0  */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_HXT;
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;    /* enable TIMER0 clock                  */
    TIMER0->CTL = 0;                   /* disable timer                                  */
    TIMER0->INTSTS = (TIMER_INTSTS_TWKF_Msk | TIMER_INTSTS_TIF_Msk);  /* clear interrupt status */
    TIMER0->CMP = 0xFFFFFE;            /* maximum time                                   */
    TIMER0->CNT = 0;                   /* clear timer counter                            */
    /* start timer */
    TIMER0->CTL = (11 << TIMER_CTL_PSC_Pos) | TIMER_ONESHOT_MODE | TIMER_CTL_CNTEN_Msk;
}

uint32_t  get_timer0_counter()
{
    return TIMER0->CNT;
}


void RecevieAndSendBack_Callback()
{
    PA4 ^= 1;
    UART_Read(u32TestPort, &u8RxData[g_u32RxIndex++], 1);
}

/*===== for Tx and Rx data =====*/


void RecevieAndSendBack(E_UART_PORT u32Port)
{
    uint32_t u32TestBR;
    uint32_t u32TimeOutCnt;
    UART_T * tUART;
    tUART = (UART_T *)(UART0_BASE + u32Port);

    printf("=== Get FW from PC ===\n");
    printf("[PB6 : UART1 RX] \n");
    printf("[PB7 : UART1 TX] \n\n");
    printf(" Connect PC<-->");
    switch(u32Port)
    {
        case UART_PORT0:
            printf("UART0.\n");
            u32TestPort = UART0;
            break;
        case UART_PORT1:
            printf("UART1.\n");
            u32TestPort = UART1;
            break;
        case UART_PORT2:
            printf("UART2.\n");
            u32TestPort = UART2;
            break;
        case UART_PORT3:
            printf("UART3.\n");
            u32TestPort = UART3;
            break;
        case UART_PORT4:
            printf("UART4.\n");
            u32TestPort = UART4;
            break;
        case UART_PORT5:
            printf("UART5.\n");
            u32TestPort = UART5;
            break;
    }
#if !defined(NO_LINE_DEBUG_HINT)
    printf(" Enter any key to continue.\n\n");
    GetChar();
#else
    printf("\n");
#endif

    /* setting before test */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TXEMPTYF_RXIDLE(DEBUG_PORT)
        if(--u32TimeOutCnt == 0) break;

    BackupCLKSetting();
    BackupUARTSetting(u32Port);
    UART_Init(tUART, UART_CLK_SRC_HXT, UART_CLK_DIV(1), 0);

    /* baud rate setting */
    tUART->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    u32TestBR = 115200;

    /* Enable Rx ready interrupt */
    DrvUART_EnableInt(u32Port, DRVUART_RDAINT, (PFN_DRVUART_CALLBACK*)RecevieAndSendBack_Callback);

    /* Prepare Rx data */
    for(g_u32RxIndex = 0; g_u32RxIndex < TXBUFSIZE; g_u32RxIndex++)
    {
        u8RxData[g_u32RxIndex] = 0xFF;
    }

    g_u32RxIndex = 0;
    printf("Set BaudRate=%d, Wait Receive %dK data\n", u32TestBR, TXBUFSIZE >> 10);
    while(g_u32RxIndex < TXBUFSIZE); /* Wait Receive enough char */
    printf("total [%d] bytes \n", g_u32RxIndex);

    /* Disable Rx ready interrupt */
    DrvUART_DisableInt(u32Port, DRVUART_RDAINT);

    /* end of test */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TXEMPTYF_RXIDLE(DEBUG_PORT)
        if(--u32TimeOutCnt == 0) break;
    UART_WAIT_TXEMPTYF_RXIDLE(tUART);
    RestoreCLKSetting();
    RestoreUARTSetting(u32Port);

}



int32_t main(void)
{

    uint32_t u32Loop;                  /* loop counter                                   */


    uint32_t u32ch, u32i,  u32RoBase = 0x0, u32Sum = 0;
    uint32_t u32TimeOutCnt;
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);
    UART_Open(UART1, 115200);

    /*-----------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                        */
    /*-----------------------------------------------------------------------------------*/

    printf("+---------------------+ \n");
    printf("|    Firmware [1]     + \n");
    printf("+---------------------+ \n");

    SYS_UnlockReg();                   /* Unlock register lock protect                   */

    FMC_Open();                        /* Enable FMC ISP function                        */
    FMC_ENABLE_AP_UPDATE();            /* Enable FMC erase/program APROM                 */
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;

    NVIC_EnableIRQ(WDT_IRQn);

    if(u32RoBase == 0x0)
        printf("===== Boot code ===== \n");
    else
        printf("===== boot from [0x%x] ===== \n", u32RoBase);


    FMC_Erase(BANK0_PAGE_CRC_BASE);

    u32Sum = func_crc32(BANK0_FW_BASE, BANK0_FW_SIZE);
    printf("FW check sum = 0x%8x\n", u32Sum);

    /* Write CRC */
    if(FMC_Read(BANK0_FW_CRC_BASE) == 0xFFFFFFFF)
        FMC_Write(BANK0_FW_CRC_BASE, u32Sum);

    /* Write version number */
    if(FMC_Read(BANK0_FW_VER_BASE) == 0xFFFFFFFF)
        FMC_Write(BANK0_FW_VER_BASE, 0x1101);

    /* Write CRC for each Page */
    for(u32i = 0; u32i < BANK0_FW_SIZE / TMP_PAGE_SIZE; u32i++)
    {
        u32Sum = func_crc32(BANK0_FW_BASE + u32i * TMP_PAGE_SIZE, TMP_PAGE_SIZE);
        printf("FW page[%d] check sum = 0x%8x\n", u32i, u32Sum);
        if(FMC_Read(BANK0_PAGE_CRC_BASE + u32i * 4) == 0xFFFFFFFF)
            FMC_Write(BANK0_PAGE_CRC_BASE + u32i * 4, u32Sum);
    }


    while(1)
    {
        printf("\nDownload new FW?[y/n]\n");
        u32ch = getchar();
        if(u32ch == 'y')
        {

            RecevieAndSendBack(UART_PORT1);
            printf("\nBank0 processing, downloaad data to bank1.\n");
            g_i8DbState = DB_STATE_DONE;          /* dual bank program state idle                   */

            enable_sys_tick(1000);
            start_timer0();

            g_u32DbAddr   = BANK1_FW_BASE;          /* Dual bank background program address           */
            g_u32DbLength = BANK1_FW_SIZE;          /* Dual bank background length                    */
            g_i8DbState   = DB_STATE_START;         /* Start background dual bank program             */

            enable_sys_tick(1000);
            start_timer0();

            for(u32Loop = 0; u32Loop < CRC32_LOOP_CNT; u32Loop++)
            {
                func_crc32(0x0, 0x10000);      /* Calculate 64KB CRC32 value, just to consume CPU time  */
            }

            while(g_i8DbState != DB_STATE_DONE) ;
            printf("\nFW download completed!!\n");
            /*
             *  Verify ...
             */
            FMC_Erase(BANK1_FW_CRC_BASE);
            u32Sum = func_crc32(BANK1_FW_BASE, BANK1_FW_SIZE);

            /* Write CRC */
            if(FMC_Read(BANK1_FW_CRC_BASE) == 0xFFFFFFFF)
                FMC_Write(BANK1_FW_CRC_BASE, u32Sum);

            /* Write version number */
            if(FMC_Read(BANK1_FW_VER_BASE) == 0xFFFFFFFF)
                FMC_Write(BANK1_FW_VER_BASE, 0x2001);

            /* Write CRC for each Page */
            for(u32i = 0; u32i < BANK1_FW_SIZE / TMP_PAGE_SIZE; u32i++)
            {
                u32Sum = func_crc32(BANK1_FW_BASE + u32i * TMP_PAGE_SIZE, TMP_PAGE_SIZE);
                printf("FW page[%d] check sum = 0x%8x\n", u32i, u32Sum);
                if(FMC_Read(BANK1_PAGE_CRC_BASE + u32i * 4) == 0xFFFFFFFF)
                    FMC_Write(BANK1_PAGE_CRC_BASE + u32i * 4, u32Sum);
            }

        }
        else
        {
            printf("\nBank0 processing, no FW loading.\n");

            SelfTest();
            printf("\nAny key to rest CPU.\n");
            getchar();

            FMC->ISPADDR = BOOT_BASE;
            /* Set VECMAP */
            FMC->ISPCMD = 0x2E;
            FMC->ISPTRG = 1;
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(FMC->ISPTRG)
            {
                if(--u32TimeOutCnt == 0)
                    break;
            }
            SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
        }
    }

}
/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/


