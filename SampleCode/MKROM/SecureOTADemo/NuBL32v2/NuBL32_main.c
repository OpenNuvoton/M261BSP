/**************************************************************************//**
 * @file     NuBL32_main.c
 * @version  V1.00
 * @brief    Executing in NuBL32.
 *
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <arm_cmse.h>
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "NuBL_common.h"
#include "ota.h"
#include "NuBL2lib.h"

#if (OTA_UPGRADE_FROM_SD)
#include "diskio.h"
#include "ff.h"
#endif

#define PLL_CLOCK       64000000

volatile ISP_INFO_T     g_ISPInfo = {0};

void BL32_OTA_Start()
{
    SYS_UnlockReg();
    FMC_Open();

    printf("BL32_OTA_Start\n");
    if (OTA_TaskProcess() == 0)
    {
    #if (OTA_UPGRADE_FROM_SD)
        printf("check OTA status: SYS:%d\n", FMC_Read(SYS_FW_OTA_STATUS_BASE));
        if (FMC_Read(SYS_FW_OTA_STATUS_BASE) == 1)
        {
            printf("BL32_OTA_done\n");
//            SYS_ResetChip();
//            while(1);
        }
    #else
        printf("check OTA status: %d\n", FMC_Read(OTA_STATUS_BASE));
        if (FMC_Read(OTA_STATUS_BASE) == 1)
        {
            printf("BL32_OTA_done\n");
//            SYS_ResetChip();
//            while(1);
        }    
    #endif
    }
}

/*----------------------------------------------------------------------------
  LED control function
 *----------------------------------------------------------------------------*/
int32_t LED_On(void)
{
    printf("LED On\n");
    PB10 = 0;
    return 1;
}

int32_t LED_Off(void)
{
    printf("LED Off\n");
    PB10 = 1;
    return 1;
}


/*----------------------------------------------------------------------------
  SysTick IRQ Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    static uint32_t u32Ticks;
    static uint32_t u32WDTTicks;
    static uint32_t u32LedTicks;

    if(u32WDTTicks > 800) /* 8s*/
    {
        WDT_RESET_COUNTER();
        u32WDTTicks = 0;
    }

    switch(u32LedTicks++)
    {
        case   0:
            LED_On();
            break;
        case 800:
            LED_Off();
            break;

        default:
            if(u32LedTicks > 1600)
            {
                u32LedTicks = 0;
            }
            break;
    }

//    printf("Tick\n");
    if (OTA_SysTickProcess(u32Ticks))
        u32Ticks = 0;
    else
        u32Ticks++;

    u32WDTTicks++;
}

void UART3_IRQHandler(void)
{
//    printf("UART3\n");
    OTA_WiFiProcess();
}


/**
 * @brief       GPIO PA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PA default IRQ, declared in startup_M2351.s.
 */
void GPA_IRQHandler(void)
{
    int32_t i32Status;
    /* To check if PA.3 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PA, BIT3))
    {
        GPIO_CLR_INT_FLAG(PA, BIT3);
        printf("PA.3 INT occurred.\n");
        i32Status = OTA_ForceUpdate();
        if (i32Status)
        {
            printf("Force update firmware was failed(0x%x).\n", i32Status);
        }
    }
    else
    {
        /* Un-expected interrupt. Just clear all PA interrupts */
        PA->INTSRC = PA->INTSRC;
        printf("Un-expected interrupts.\n");
    }
}

#if (OTA_UPGRADE_FROM_SD)
/**
  * @brief SD Host interrupt process
  * @param      None
  * @return     None
  * @details    SD Host interrupt process
  */
void SDH0_IRQHandler(void)
{
    OTA_SDH_Process();
}
#endif


void SYS_Init(void)
{
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

    /* Select UART module clock source as HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & (~(UART0_RXD_PB12_Msk | UART0_TXD_PB13_Msk))) | UART0_RXD_PB12 | UART0_TXD_PB13;
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open((UART_T *)DEBUG_PORT, 115200);
}

void UART3_Init()
{
    CLK->APBCLK0 |= CLK_APBCLK0_UART3CKEN_Msk;
    CLK->CLKSEL3 = (CLK->CLKSEL3 & (~CLK_CLKSEL3_UART3SEL_Msk)) | CLK_CLKSEL3_UART3SEL_HIRC;

    UART3->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    UART3->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

    /* Set multi-function pins for UART3 RXD and TXD */
    SYS->GPD_MFPL = (SYS->GPD_MFPL & (~(UART3_RXD_PD0_Msk | UART3_TXD_PD1_Msk))) | UART3_RXD_PD0 | UART3_TXD_PD1;
}

#if (OTA_UPGRADE_FROM_SD)
void SD_Init()
{
    /* select multi-function pins */
    SYS->GPE_MFPL &= ~(SYS_GPE_MFPL_PE2MFP_Msk | SYS_GPE_MFPL_PE3MFP_Msk | SYS_GPE_MFPL_PE4MFP_Msk | SYS_GPE_MFPL_PE5MFP_Msk |
                       SYS_GPE_MFPL_PE6MFP_Msk | SYS_GPE_MFPL_PE7MFP_Msk);
    SYS->GPD_MFPH &= ~SYS_GPD_MFPH_PD13MFP_Msk;
    SYS->GPE_MFPL |= (SYS_GPE_MFPL_PE2MFP_SD0_DAT0 | SYS_GPE_MFPL_PE3MFP_SD0_DAT1 | SYS_GPE_MFPL_PE4MFP_SD0_DAT2 | SYS_GPE_MFPL_PE5MFP_SD0_DAT3 |
                      SYS_GPE_MFPL_PE6MFP_SD0_CLK | SYS_GPE_MFPL_PE7MFP_SD0_CMD);
    SYS->GPD_MFPH |= SYS_GPD_MFPH_PD13MFP_SD0_nCD;

    //SD_PWR: PF9 - it should be pulled low to enalbed the pull-high resistor for SDIO pins(NuTiny-M2351)
    SYS->GPF_MFPH = (SYS->GPF_MFPH & (~SYS_GPF_MFPH_PF9MFP_Msk));
    GPIO_SetMode(PF, BIT9, GPIO_MODE_OUTPUT);
    PF9 = 0;

    /* Select IP clock source */
    CLK_SetModuleClock(SDH0_MODULE, CLK_CLKSEL0_SDH0SEL_PLL, CLK_CLKDIV0_SDH0(4));
    /* Enable IP clock */
    CLK_EnableModuleClock(SDH0_MODULE);

    /* Enable NVIC SDH0 IRQ */
    NVIC_EnableIRQ(SDH0_IRQn);

}
#endif

void GPIO_init(void)
{
    /* Configure PA.3 as Input mode and enable interrupt by rising edge trigger */
    GPIO_SetMode(PA, BIT3, GPIO_MODE_INPUT);
    GPIO_EnableInt(PA, 3, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPA_IRQn);
    
    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(PA, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PA, BIT3);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t    count = 0;
    uint32_t    u32FwVer = 0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\n[HCLK %d Hz] (%s, %s)\n", SystemCoreClock, __DATE__, __TIME__);
    printf("+--------------------------------+\n");
    printf("|    M261 NuBL32 Sample Code    |\n");
    printf("+--------------------------------+\n\n");
    if (OTA_GetBLxFwVer((uint32_t *)&u32FwVer, 0) == 0)
        printf("NuBL32 Firmware Ver: 0x%08x\n\n", u32FwVer);
    else
        printf("NuBL32 Firmware Ver: N/A\n\n");
    CLK_SysTickDelay(200000);

{
    extern const FW_INFO_T g_InitialFWinfo;
    uint32_t cfg = g_InitialFWinfo.mData.u32AuthCFGs;
    printf("\n[AuthCFG: 0x%08x]\n", cfg);
}

    /* To check if system has been reset by WDT time-out reset or not */
    if(WDT_GET_RESET_FLAG() == 1)
    {
        WDT_CLEAR_RESET_FLAG();

        printf("*** System has been reset by WDT time-out event ***\n\n");
    }

    GPIO_init();
    
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();
    
    /* Init GPIO Port B for LED control */
    GPIO_SetMode(PB, BIT10, GPIO_MODE_OUTPUT);

    /* Generate Systick interrupt each 10 ms */
    SysTick_Config(SystemCoreClock / 100);

    printf("OTA and WIfi init...\n");
    /* init OTA */
    OTA_Init(__HSI, (ISP_INFO_T *)&g_ISPInfo);

    UART3_Init();

#if (OTA_UPGRADE_FROM_SD)
    /* Init SD */
    SD_Init();
#endif

    SystemCoreClockUpdate();
    /* Generate Systick interrupt each 1 ms */
    SysTick_Config(SystemCoreClock / 1000);

    BL32_OTA_Start();

    while(1) {}
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
