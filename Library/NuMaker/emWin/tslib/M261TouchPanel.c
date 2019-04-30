
#include "NuMicro.h"

#include "M261TouchPanel.h"



int Init_TouchPanel(void)
{
    /* Enable peripheral clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

    /* Init ADC for TP */
    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    return 1;
}

static volatile    uint32_t    g_u32AdcIntFlag_TP;

/*-----------------------------------------------*/
// ADC01 ISR
//
/*-----------------------------------------------*/
void EADC1_IRQHandler(void)
{
    /* Clear the A/D ADINT1 interrupt flag */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);

    g_u32AdcIntFlag_TP = 1;

}

/*-----------------------------------------------*/
// Get X Position from Touch Panel (ADC input)
//
/*-----------------------------------------------*/
uint16_t Get_TP_X(void)
{
    uint16_t    x_adc_in;

    /*=== Get X from ADC input ===*/
    GPIO_SetMode(PB, BIT4, GPIO_MODE_OUTPUT);   // XR
    GPIO_SetMode(PB, BIT5, GPIO_MODE_INPUT);    // YD
    GPIO_SetMode(PB, BIT6, GPIO_MODE_OUTPUT);   // XL
    PB4 = 1;
    PB6 = 0;

    /* Configure the GPB7 ADC analog input pins.  */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB7MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk);
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB7MFP_EADC0_CH7;

    /* Disable the GPB7 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT7);

    /* Configure the sample module 1 for analog input channel 7 and software trigger source.*/
    EADC_ConfigSampleModule(EADC, 1, EADC_SOFTWARE_TRIGGER, 7); // YU

    /* Clear the A/D ADINT1 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);

    /* Enable the sample module 1 interrupt.  */
    EADC_ENABLE_INT(EADC, BIT1);    //Enable sample module A/D ADINT1 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 1, BIT1);    //Enable sample module 1 interrupt.
    NVIC_EnableIRQ(EADC1_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module 1 to start A/D conversion */
    g_u32AdcIntFlag_TP = 0;
    EADC_START_CONV(EADC, BIT1);

    /* Wait ADC interrupt (g_u32AdcIntFlag_TP will be set at IRQ_Handler function) */
    while(g_u32AdcIntFlag_TP == 0);
    x_adc_in = EADC_GET_CONV_DATA(EADC, 1);
    return x_adc_in;

}


/*-----------------------------------------------*/
// Get Y Position from Touch Panel (ADC input)
//
/*-----------------------------------------------*/
uint16_t Get_TP_Y(void)
{
    uint16_t    y_adc_in;

    /*=== Get Y from ADC input ===*/
    GPIO_SetMode(PB, BIT7, GPIO_MODE_OUTPUT);   // YU
    GPIO_SetMode(PB, BIT5, GPIO_MODE_OUTPUT);   // YD
    GPIO_SetMode(PB, BIT6, GPIO_MODE_INPUT);    // XL
    PB7 = 1;
    PB5 = 0;

    /* Configure the GPB4 ADC analog input pins.  */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB7MFP_Msk | SYS_GPB_MFPL_PB4MFP_Msk);
    SYS->GPB_MFPL |= SYS_GPB_MFPL_PB4MFP_EADC0_CH4;

    /* Disable the GPB4 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT4);

    /* Configure the sample module 2 for analog input channel 4 and software trigger source.*/
    EADC_ConfigSampleModule(EADC, 2, EADC_SOFTWARE_TRIGGER, 4); // XR

    /* Clear the A/D ADINT1 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF1_Msk);

    /* Enable the sample module 2 interrupt.  */
    EADC_ENABLE_INT(EADC, BIT2);    //Enable sample module A/D ADINT1 interrupt.
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 1, BIT2);    //Enable sample module 2 interrupt.
    NVIC_EnableIRQ(EADC1_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module 2 to start A/D conversion */
    g_u32AdcIntFlag_TP = 0;
    EADC_START_CONV(EADC, BIT2);

    /* Wait ADC interrupt (g_u32AdcIntFlag_TP will be set at IRQ_Handler function) */
    while(g_u32AdcIntFlag_TP == 0);
    y_adc_in = EADC_GET_CONV_DATA(EADC, 2);
    return y_adc_in;

}

int Read_TouchPanel(int *x, int *y)
{
    *x = Get_TP_X();
    *y = Get_TP_Y();
    if(((*x & 0x0F00) >= 0x0F00) || ((*y & 0x0F00) >= 0x0F00))
        return 0;
    else
        return 1;
}

int Uninit_TouchPanel(void)
{
    return 1;
}

int Check_TouchPanel(void)
{
    return 0;   //Pen up;
}

