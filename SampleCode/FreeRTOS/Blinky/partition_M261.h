/**************************************************************************//**
 * @file     partition_TC8234.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/12/08 6:56p $
 * @brief    SAU configuration for secure/nonsecure region settings.
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef PARTITION_TC8234
#define PARTITION_TC8234

/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <e>Secure Attribute Unit (SAU) Control
*/
#define SAU_INIT_CTRL 1

/*
//   <q> Enable SAU
//   <i> To enable Secure Attribute Unit (SAU).
*/
#define SAU_INIT_CTRL_ENABLE 1

/*
//   <o> All Memory Attribute When SAU is disabled
//     <0=> All Memory is Secure
//     <1=> All Memory is Non-Secure
//   <i> To set the ALLNS bit in SAU CTRL.
//   <i> When all Memory is Non-Secure (ALLNS is 1), IDAU can override memory map configuration.
*/
#define SAU_INIT_CTRL_ALLNS  0

/*
// </e>
*/

/*
// <h>Enable and Set Secure/Non-Secure region
*/
#define SAU_REGIONS_MAX   8                 /* Max. number of SAU regions */

/*
//   <e>SAU Region 0
//   <i> Setup SAU Region 0
*/
#define SAU_INIT_REGION0    1
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START0     0x00000000      /* start address of SAU region 0 */
/*
//     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END0       0x00040000      /* end address of SAU region 0 */
/*
//     <o>Region is
//         <0=>Non-Secure
//         <1=>Secure, Non-Secure Callable
*/
#define SAU_INIT_NSC0       1
/*
//   </e>
*/

/*
//   <e>SAU Region 1
//   <i> Setup SAU Region 1
*/
#define SAU_INIT_REGION1    1
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START1     0x10040000
/*
//     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END1       0x1007FFFF
/*
//     <o>Region is
//         <0=>Non-Secure
//         <1=>Secure, Non-Secure Callable
*/
#define SAU_INIT_NSC1       0
/*
//   </e>
*/

/*
//   <e>SAU Region 2
//   <i> Setup SAU Region 2
*/
#define SAU_INIT_REGION2    1
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START2     0x2000F000
/*
//     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END2       0x2000FFFF
/*
//     <o>Region is
//         <0=>Non-Secure
//         <1=>Secure, Non-Secure Callable
*/
#define SAU_INIT_NSC2       1
/*
//   </e>
*/

/*
//   <e>SAU Region 3
//   <i> Setup SAU Region 3
*/
#define SAU_INIT_REGION3    1
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START3     0x30010000
/*
//     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END3       0x3001FFFF
/*
//     <o>Region is
//         <0=>Non-Secure
//         <1=>Secure, Non-Secure Callable
*/
#define SAU_INIT_NSC3       0
/*
//   </e>
*/

/*
//   <e>SAU Region 4
//   <i> Setup SAU Region 4
*/
#define SAU_INIT_REGION4    1
/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START4     0x50000000      /* start address of SAU region 4 */

/*
//     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END4       0x5FFFFFFF      /* end address of SAU region 4 */

/*
//     <o>Region is
//         <0=>Non-Secure
//         <1=>Secure, Non-Secure Callable
*/
#define SAU_INIT_NSC4       0
/*
//   </e>
*/

/*
//   <e>SAU Region 5
//   <i> Setup SAU Region 5
*/
#define SAU_INIT_REGION5    1

/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START5     0x00807C00

/*
//     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END5       0x00807FFF

/*
//     <o>Region is
//         <0=>Non-Secure
//         <1=>Secure, Non-Secure Callable
*/
#define SAU_INIT_NSC5       1
/*
//   </e>
*/

/*
//   <e>SAU Region 6
//   <i> Setup SAU Region 6
*/
#define SAU_INIT_REGION6    0

/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START6     0x00000000

/*
//     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END6       0x00000000

/*
//     <o>Region is
//         <0=>Non-Secure
//         <1=>Secure, Non-Secure Callable
*/
#define SAU_INIT_NSC6       0
/*
//   </e>
*/

/*
//   <e>SAU Region 7
//   <i> Setup SAU Region 7
*/
#define SAU_INIT_REGION7    0

/*
//     <o>Start Address <0-0xFFFFFFE0>
*/
#define SAU_INIT_START7     0x00000000

/*
//     <o>End Address <0x1F-0xFFFFFFFF>
*/
#define SAU_INIT_END7       0x00000000

/*
//     <o>Region is
//         <0=>Non-Secure
//         <1=>Secure, Non-Secure Callable
*/
#define SAU_INIT_NSC7       0
/*
//   </e>
*/

/*
// </h>
*/

/*
// <e>Setup behaviour of Sleep and Exception Handling
*/
#define SCB_CSR_AIRCR_INIT  1

/*
//   <o> Deep Sleep can be enabled by
//     <0=>Secure and Non-Secure state
//     <1=>Secure state only
//   <i> Value for SCB->CSR register bit DEEPSLEEPS
*/
#define SCB_CSR_DEEPSLEEPS_VAL  0

/*
//   <o>System reset request accessible from
//     <0=> Secure and Non-Secure state
//     <1=> Secure state only
//   <i> Value for SCB->AIRCR register bit SYSRESETREQS
*/
#define SCB_AIRCR_SYSRESETREQS_VAL  0

/*
//   <o>Priority of Non-Secure exceptions is
//     <0=> Not altered
//     <1=> Lowered to 0x80-0xFF
//   <i> Value for SCB->AIRCR register bit PRIS
*/
#define SCB_AIRCR_PRIS_VAL      0

/*
//   <o>BusFault, HardFault, and NMI target
//     <0=> Secure state
//     <1=> Non-Secure state
//   <i> Value for SCB->AIRCR register bit BFHFNMINS
*/
#define SCB_AIRCR_BFHFNMINS_VAL 0

/*
// </e>
*/


/*
// <h>Assign Interrupt to Secure or Non-secure Vector
*/


/*
    Initialize ITNS 0 (Interrupts 0..31)
*/
#define NVIC_INIT_ITNS0    1
/*
//   <o.0>  BODOUT                <0=> Secure <1=> Non-Secure
//   <o.1>  IRC                   <0=> Secure <1=> Non-Secure
//   <o.2>  PWRWU_                <0=> Secure <1=> Non-Secure
//   <o.3>  SRAM_PERR             <0=> Secure <1=> Non-Secure
//   <o.4>  CLKFAIL               <0=> Secure <1=> Non-Secure

//   <o.6>  RTC                   <0=> Secure <1=> Non-Secure
//   <o.7>  TAMPER                <0=> Secure <1=> Non-Secure
//   <o.8>  WDT                   <0=> Secure <1=> Non-Secure
//   <o.9>  WWDT                  <0=> Secure <1=> Non-Secure
//   <o.10> EINT0                 <0=> Secure <1=> Non-Secure
//   <o.11> EINT1                 <0=> Secure <1=> Non-Secure
//   <o.12> EINT2                 <0=> Secure <1=> Non-Secure
//   <o.13> EINT3                 <0=> Secure <1=> Non-Secure
//   <o.14> EINT4                 <0=> Secure <1=> Non-Secure
//   <o.15> EINT5                 <0=> Secure <1=> Non-Secure
//   <o.16> GPA                   <0=> Secure <1=> Non-Secure
//   <o.17> GPB                   <0=> Secure <1=> Non-Secure
//   <o.18> GPC                   <0=> Secure <1=> Non-Secure
//   <o.19> GPD                   <0=> Secure <1=> Non-Secure
//   <o.20> GPE                   <0=> Secure <1=> Non-Secure
//   <o.21> GPF               <0=> Secure <1=> Non-Secure
//   <o.22> SPI0              <0=> Secure <1=> Non-Secure
//   <o.23> SPI1              <0=> Secure <1=> Non-Secure
//   <o.24> BRAKE0            <0=> Secure <1=> Non-Secure
//   <o.25> PWM0_P0           <0=> Secure <1=> Non-Secure
//   <o.26> PWM0_P1           <0=> Secure <1=> Non-Secure
//   <o.27> PWM0_P2           <0=> Secure <1=> Non-Secure
//   <o.28> BRAKE1            <0=> Secure <1=> Non-Secure
//   <o.29> PWM1_P0           <0=> Secure <1=> Non-Secure
//   <o.30> PWM1_P1           <0=> Secure <1=> Non-Secure
//   <o.31> PWM1_P2           <0=> Secure <1=> Non-Secure
//
*/
#define NVIC_INIT_ITNS0_VAL      0x0

/*
    Initialize ITNS 1 (Interrupts 0..31)
*/
#define NVIC_INIT_ITNS1    1
/*
//   <o.0>  TMR0              <0=> Secure <1=> Non-Secure
//   <o.1>  TMR1              <0=> Secure <1=> Non-Secure
//   <o.2>  TMR2              <0=> Secure <1=> Non-Secure
//   <o.3>  TMR3              <0=> Secure <1=> Non-Secure
//   <o.4>  UART0             <0=> Secure <1=> Non-Secure
//   <o.5>  UART1             <0=> Secure <1=> Non-Secure
//   <o.6>  I2C0              <0=> Secure <1=> Non-Secure
//   <o.7>  I2C1              <0=> Secure <1=> Non-Secure
// PDMA0 is secure only
//   <o.9>  DAC               <0=> Secure <1=> Non-Secure
//   <o.10> EADC0             <0=> Secure <1=> Non-Secure
//   <o.11> EADC1             <0=> Secure <1=> Non-Secure
//   <o.12> ACMP01            <0=> Secure <1=> Non-Secure

//   <o.14> EADC2             <0=> Secure <1=> Non-Secure
//   <o.15> EADC3             <0=> Secure <1=> Non-Secure
//   <o.16> UART2             <0=> Secure <1=> Non-Secure
//   <o.17> UART3             <0=> Secure <1=> Non-Secure

//   <o.19> SPI2              <0=> Secure <1=> Non-Secure
//   <o.20> SPI3              <0=> Secure <1=> Non-Secure
//   <o.21> USBD              <0=> Secure <1=> Non-Secure
//   <o.22> USBH              <0=> Secure <1=> Non-Secure
//   <o.23> USBOTG            <0=> Secure <1=> Non-Secure
//   <o.24> CAN0              <0=> Secure <1=> Non-Secure

//   <o.26> SC0               <0=> Secure <1=> Non-Secure
//   <o.27> SC1               <0=> Secure <1=> Non-Secure
//   <o.28> SC2               <0=> Secure <1=> Non-Secure

//   <o.30> SPI4              <0=> Secure <1=> Non-Secure

//
*/
#define NVIC_INIT_ITNS1_VAL      0x0

/*
    Initialize ITNS 2 (Interrupts 0..31)
*/
#define NVIC_INIT_ITNS2    1
/*
//   <o.0>  SDH0              <0=> Secure <1=> Non-Secure



//   <o.4>  I2S0              <0=> Secure <1=> Non-Secure

//   <o.6>  OPA0              <0=> Secure <1=> Non-Secure
//   <o.7>  CRYPTO                <0=> Secure <1=> Non-Secure
//   <o.8>  GPG               <0=> Secure <1=> Non-Secure

//   <o.10> UART4             <0=> Secure <1=> Non-Secure
//   <o.11> UART5             <0=> Secure <1=> Non-Secure
//   <o.12> USCI0             <0=> Secure <1=> Non-Secure
//   <o.13> USCI1             <0=> Secure <1=> Non-Secure
//   <o.14> BPWM0             <0=> Secure <1=> Non-Secure
//   <o.15> BPWM1             <0=> Secure <1=> Non-Secure

//   <o.17> ICAP              <0=> Secure <1=> Non-Secure
//   <o.18> I2C2              <0=> Secure <1=> Non-Secure

//   <o.20> QEI0              <0=> Secure <1=> Non-Secure
//   <o.21> QEI1              <0=> Secure <1=> Non-Secure
//   <o.22> ECAP0             <0=> Secure <1=> Non-Secure
//   <o.23> ECAP1             <0=> Secure <1=> Non-Secure


//   <o.26> SDH1              <0=> Secure <1=> Non-Secure

//   <o.28> USBH              <0=> Secure <1=> Non-Secure



//
*/
#define NVIC_INIT_ITNS2_VAL      0x0


/*
    Initialize ITNS 3 (Interrupts 0..31)
*/
#define NVIC_INIT_ITNS3    1
/*
//   <o.0>  SPI5              <0=> Secure <1=> Non-Secure
//   <o.1>  DSRC              <0=> Secure <1=> Non-Secure
//   <o.2>  PDMA1             <0=> Secure <1=> Non-Secure
//   <o.3>  SCU               <0=> Secure <1=> Non-Secure
//   <o.4>  LCD               <0=> Secure <1=> Non-Secure
//   <o.5>  TRNG              <0=> Secure <1=> Non-Secure
*/
#define NVIC_INIT_ITNS3_VAL      0x0



/*
// </h>
*/



/*
    max 128 SAU regions.
    SAU regions are defined in partition.h
 */

#define SAU_INIT_REGION(n) \
    SAU->RNR  =  (n                                     & SAU_RNR_REGION_Msk); \
    SAU->RBAR =  (SAU_INIT_START##n                     & SAU_RBAR_BADDR_Msk); \
    SAU->RLAR =  (SAU_INIT_END##n                       & SAU_RLAR_LADDR_Msk) | \
                ((SAU_INIT_NSC##n << SAU_RLAR_NSC_Pos)  & SAU_RLAR_NSC_Msk)   | 1U

/**
  \brief   Setup a SAU Region
  \details Writes the region information contained in SAU_Region to the
           registers SAU_RNR, SAU_RBAR, and SAU_RLAR
 */
__STATIC_INLINE void TZ_SAU_Setup(void)
{

#if defined (__SAU_PRESENT) && (__SAU_PRESENT == 1U)

#if defined (SAU_INIT_REGION0) && (SAU_INIT_REGION0 == 1U)
    SAU_INIT_REGION(0);
#endif

#if defined (SAU_INIT_REGION1) && (SAU_INIT_REGION1 == 1U)
    SAU_INIT_REGION(1);
#endif

#if defined (SAU_INIT_REGION2) && (SAU_INIT_REGION2 == 1U)
    SAU_INIT_REGION(2);
#endif

#if defined (SAU_INIT_REGION3) && (SAU_INIT_REGION3 == 1U)
    SAU_INIT_REGION(3);
#endif

#if defined (SAU_INIT_REGION4) && (SAU_INIT_REGION4 == 1U)
    SAU_INIT_REGION(4);
#endif

#if defined (SAU_INIT_REGION5) && (SAU_INIT_REGION5 == 1U)
    SAU_INIT_REGION(5);
#endif

#if defined (SAU_INIT_REGION6) && (SAU_INIT_REGION6 == 1U)
    SAU_INIT_REGION(6);
#endif

#if defined (SAU_INIT_REGION7) && (SAU_INIT_REGION7 == 1U)
    SAU_INIT_REGION(7);
#endif

    /* repeat this for all possible SAU regions */


#if defined (SAU_INIT_CTRL) && (SAU_INIT_CTRL == 1U)
    SAU->CTRL = ((SAU_INIT_CTRL_ENABLE << SAU_CTRL_ENABLE_Pos) & SAU_CTRL_ENABLE_Msk) |
                ((SAU_INIT_CTRL_ALLNS  << SAU_CTRL_ALLNS_Pos)  & SAU_CTRL_ALLNS_Msk)   ;
#endif

#endif /* defined (__SAU_PRESENT) && (__SAU_PRESENT == 1U) */

#if defined (SCB_CSR_AIRCR_INIT) && (SCB_CSR_AIRCR_INIT == 1U)
    SCB->SCR   = (SCB->SCR   & ~(SCB_SCR_SLEEPDEEPS_Msk)) |
                 ((SCB_CSR_DEEPSLEEPS_VAL     << SCB_SCR_SLEEPDEEPS_Pos)     & SCB_SCR_SLEEPDEEPS_Msk);

//    SCB->AIRCR = (SCB->AIRCR & ~(SCB_AIRCR_SYSRESETREQS_Msk | SCB_AIRCR_BFHFNMINS_Msk |  SCB_AIRCR_PRIS_Msk)) |
//                 ((SCB_AIRCR_SYSRESETREQS_VAL << SCB_AIRCR_SYSRESETREQS_Pos) & SCB_AIRCR_SYSRESETREQS_Msk) |
//                 ((SCB_AIRCR_BFHFNMINS_VAL    << SCB_AIRCR_BFHFNMINS_Pos)    & SCB_AIRCR_BFHFNMINS_Msk)    |
//                 ((SCB_AIRCR_PRIS_VAL         << SCB_AIRCR_PRIS_Pos)         & SCB_AIRCR_PRIS_Msk);

    SCB->AIRCR = (0x05FA << 16) |
                 ((SCB_AIRCR_SYSRESETREQS_VAL << SCB_AIRCR_SYSRESETREQS_Pos) & SCB_AIRCR_SYSRESETREQS_Msk) |
                 ((SCB_AIRCR_BFHFNMINS_VAL    << SCB_AIRCR_BFHFNMINS_Pos)    & SCB_AIRCR_BFHFNMINS_Msk)    |
                 ((SCB_AIRCR_PRIS_VAL         << SCB_AIRCR_PRIS_Pos)         & SCB_AIRCR_PRIS_Msk);



#endif /* defined (SCB_CSR_AIRCR_INIT) && (SCB_CSR_AIRCR_INIT == 1U) */

#if defined (SCB_ICSR_INIT) && (SCB_ICSR_INIT == 1U)
    SCB->ICSR  = (SCB->ICSR  & ~(SCB_ICSR_STTNS_Msk)) |
                 ((SCB_ICSR_STTNS_VAL         << SCB_ICSR_STTNS_Pos)         & SCB_ICSR_STTNS_Msk);
#endif /* defined (SCB_ICSR_INIT) && (SCB_ICSR_INIT == 1U) */

#if defined (NVIC_INIT_ITNS0) && (NVIC_INIT_ITNS0 == 1U)
    NVIC->ITNS[0] = NVIC_INIT_ITNS0_VAL;
#endif

#if defined (NVIC_INIT_ITNS1) && (NVIC_INIT_ITNS1 == 1U)
    NVIC->ITNS[1] = NVIC_INIT_ITNS1_VAL;
#endif

#if defined (NVIC_INIT_ITNS2) && (NVIC_INIT_ITNS2 == 1U)
    NVIC->ITNS[2] = NVIC_INIT_ITNS2_VAL;
#endif

#if defined (NVIC_INIT_ITNS3) && (NVIC_INIT_ITNS3 == 1U)
    NVIC->ITNS[3] = NVIC_INIT_ITNS3_VAL;
#endif


    /* repeat this for all possible ITNS elements */

}


/*
// <h> Peripheral Secure Attribution Configuration
*/

/*
    PNSSET0
*/
/*
// Module 0..31
//   <o.9>  USBH       <0=> Secure <1=> Non-Secure
//   <o.13>  SD0   <0=> Secure <1=> Non-Secure
//   <o.14>  SD1      <0=> Secure <1=> Non-Secure
//   <o.16>  EBI    <0=> Secure <1=> Non-Secure
//   <o.24>  PDMA1      <0=> Secure <1=> Non-Secure
*/
#define SCU_INIT_PNSSET0_VAL      0x00000001
/*
    PNSSET1
*/
/*
// Module 0..31
//   <o.17>  CRC       <0=> Secure <1=> Non-Secure
//   <o.18>  CRPT   <0=> Secure <1=> Non-Secure
*/
#define SCU_INIT_PNSSET1_VAL      0x00000000
/*
    PNSSET2
*/
/*
// Module 0..31
//   <o.1>  RTC       <0=> Secure <1=> Non-Secure
//   <o.3>  EADC   <0=> Secure <1=> Non-Secure
//   <o.5>  ACMP0      <0=> Secure <1=> Non-Secure
//   <o.6>  OPA    <0=> Secure <1=> Non-Secure
//   <o.7>  DAC      <0=> Secure <1=> Non-Secure
//   <o.8>  I2S0      <0=> Secure <1=> Non-Secure
//   <o.13>  OTG      <0=> Secure <1=> Non-Secure
//   <o.17>  TMR23      <0=> Secure <1=> Non-Secure
//   <o.24>  PWM0      <0=> Secure <1=> Non-Secure
//   <o.25>  PWM1      <0=> Secure <1=> Non-Secure
//   <o.26>  BPWM0      <0=> Secure <1=> Non-Secure
//   <o.27>  BPWM1      <0=> Secure <1=> Non-Secure
*/
#define SCU_INIT_PNSSET2_VAL      0x00000000
/*
    PNSSET3
*/
/*
// Module 0..31
//   <o.0>  SPI0       <0=> Secure <1=> Non-Secure
//   <o.1>  SPI1   <0=> Secure <1=> Non-Secure
//   <o.2>  SPI2      <0=> Secure <1=> Non-Secure
//   <o.3>  SPI3    <0=> Secure <1=> Non-Secure
//   <o.4>  SPI4      <0=> Secure <1=> Non-Secure
//   <o.5>  SPI5      <0=> Secure <1=> Non-Secure
//   <o.16>  UART0      <0=> Secure <1=> Non-Secure
//   <o.17>  UART1      <0=> Secure <1=> Non-Secure
//   <o.18>  UART2      <0=> Secure <1=> Non-Secure
//   <o.19>  UART3      <0=> Secure <1=> Non-Secure
//   <o.20>  UART4      <0=> Secure <1=> Non-Secure
//   <o.21>  UART5      <0=> Secure <1=> Non-Secure
*/
#define SCU_INIT_PNSSET3_VAL      0x00000000
/*
    PNSSET4
*/
/*
// Module 0..31
//   <o.0>  I2C0       <0=> Secure <1=> Non-Secure
//   <o.1>  I2C1   <0=> Secure <1=> Non-Secure
//   <o.2>  I2C2      <0=> Secure <1=> Non-Secure
//   <o.16>  SC0      <0=> Secure <1=> Non-Secure
//   <o.17>  SC1      <0=> Secure <1=> Non-Secure
//   <o.18>  SC2      <0=> Secure <1=> Non-Secure
*/
#define SCU_INIT_PNSSET4_VAL      0x00000000
/*
    PNSSET5
*/
/*
// Module 0..31
//   <o.0>  CAN0       <0=> Secure <1=> Non-Secure
//   <o.16>  QEI0   <0=> Secure <1=> Non-Secure
//   <o.17>  QEI1      <0=> Secure <1=> Non-Secure
//   <o.20>  ECAP0    <0=> Secure <1=> Non-Secure
//   <o.21>  ECAP1      <0=> Secure <1=> Non-Secure
//   <o.23>  DSRC    <0=> Secure <1=> Non-Secure
//   <o.24>  LCD      <0=> Secure <1=> Non-Secure
//   <o.25>  TRNG    <0=> Secure <1=> Non-Secure
*/
#define SCU_INIT_PNSSET5_VAL      0x00000000
/*
    PNSSET6
*/
/*
// Module 0..31
//   <o.0>  USBD       <0=> Secure <1=> Non-Secure
//   <o.16>  USCI0   <0=> Secure <1=> Non-Secure
//   <o.17>  USCI1      <0=> Secure <1=> Non-Secure
*/
#define SCU_INIT_PNSSET6_VAL      0x00000000
/*
// </h>
*/



/*
// <h> GPIO Secure Attribution Configuration
*/

/*
    IONSSET
*/
/*
// Bit 0..31
//   <o.0>  PA       <0=> Secure <1=> Non-Secure
//   <o.1>  PB   <0=> Secure <1=> Non-Secure
//   <o.2>  PC      <0=> Secure <1=> Non-Secure
//   <o.3>  PD    <0=> Secure <1=> Non-Secure
//   <o.4>  PE      <0=> Secure <1=> Non-Secure
//   <o.5>  PF      <0=> Secure <1=> Non-Secure
//   <o.6>  PG      <0=> Secure <1=> Non-Secure
*/
#define SCU_INIT_IONSSET_VAL      0x00000000
/*
// </h>
*/


/*
// <h> SRAM Secure Attribution Configuration
*/

/*
    SRAMNSSET
*/
/*
// Bit 0..31
//   <o.0>  0x00000000 ~ 0x00001FFF        <0=> Secure <1=> Non-Secure
//   <o.1>  0x00002000 ~ 0x00003FFF   <0=> Secure <1=> Non-Secure
//   <o.2>  0x00004000 ~ 0x00005FFF      <0=> Secure <1=> Non-Secure
//   <o.3>  0x00006000 ~ 0x00007FFF    <0=> Secure <1=> Non-Secure
//   <o.4>  0x00008000 ~ 0x00009FFF      <0=> Secure <1=> Non-Secure
//   <o.5>  0x0000A000 ~ 0x0000BFFF      <0=> Secure <1=> Non-Secure
//   <o.6>  0x0000C000 ~ 0x0000DFFF      <0=> Secure <1=> Non-Secure
//   <o.7>  0x0000E000 ~ 0x0000FFFF       <0=> Secure <1=> Non-Secure
//   <o.8>  0x00010000 ~ 0x00011FFF   <0=> Secure <1=> Non-Secure
//   <o.9>  0x00012000 ~ 0x00013FFF      <0=> Secure <1=> Non-Secure
//   <o.10> 0x00014000 ~ 0x00015FFF    <0=> Secure <1=> Non-Secure
//   <o.11> 0x00016000 ~ 0x00017FFF      <0=> Secure <1=> Non-Secure
//   <o.12> 0x00018000 ~ 0x00019FFF      <0=> Secure <1=> Non-Secure
//   <o.13> 0x0001A000 ~ 0x0001BFFF      <0=> Secure <1=> Non-Secure
//   <o.14> 0x0001C000 ~ 0x0001DFFF      <0=> Secure <1=> Non-Secure
//   <o.15> 0x0001E000 ~ 0x0001FFFF      <0=> Secure <1=> Non-Secure
*/
#define SCU_INIT_SRAMNSSET_VAL      0x00000000

/*
// </h>
*/






/**
  \brief   Setup SCU Configuration Unit
  \details

 */
__STATIC_INLINE void SCU_Setup(void)
{
    SCU->PNSSET[0] = SCU_INIT_PNSSET0_VAL;
    SCU->PNSSET[1] = SCU_INIT_PNSSET1_VAL;
    SCU->PNSSET[2] = SCU_INIT_PNSSET2_VAL;
    SCU->PNSSET[3] = SCU_INIT_PNSSET3_VAL;
    SCU->PNSSET[4] = SCU_INIT_PNSSET4_VAL;
    SCU->PNSSET[5] = SCU_INIT_PNSSET5_VAL;
    SCU->PNSSET[6] = SCU_INIT_PNSSET6_VAL;

    SCU->IONSSET = SCU_INIT_IONSSET_VAL;
    SCU->SRAMNSSET = SCU_INIT_SRAMNSSET_VAL;


}



/*
// <h> APROM Non-secure Base Address Configuration
*/

/*
    NSBA
*/
/*
//   <e> Enable Non-secure Base Address
//   <i> To check active Non-secure base address.
*/
#define FMC_INIT_NSBA          1
/*
//     <o>NSBA <0x4000-0x7FFFF>
*/
#define FMC_INIT_NSBA_VAL      0x00040000

/*
// </e>
// </h>
*/



__STATIC_INLINE void FMC_NSBA_Setup(void)
{
    /* Skip NSBA Setupt according config */
    if(FMC_INIT_NSBA == 0)
        return;

    /* Check if NSBA value with current active NSBA */
    if(SCU->FNSADDR != FMC_INIT_NSBA_VAL)
    {
        /* Unlock Protected Register */
        SYS_UnlockReg();

        /* Enable ISP and config update */
        FMC->ISPCTL = FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_CFGUEN_Msk;

        /* Config Base of NSBA */
        FMC->ISPADDR = 0x200800;

        /* Read Non-secure base address config */
        FMC->ISPCMD = FMC_ISPCMD_READ;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        while(FMC->ISPTRG);

        /* Setting NSBA when it is empty */
        if(FMC->ISPDAT == 0xfffffffful)
        {
            FMC->ISPDAT = FMC_INIT_NSBA_VAL;
            FMC->ISPCMD = FMC_ISPCMD_PROGRAM;
            FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
            while(FMC->ISPTRG);

            /* Force Chip Reset to valid new setting */
            SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
        }

        /* Fatal Error:
           FMC NSBA setting is different to FMC_INIT_NSBA_VAL.
           User must double confirm which one is wrong.

           If user need to change NSBA config of FMC, user must do Mess-erase by
           ISP or ICP.
        */
        while(1);
    }

}

#endif  /* PARTITION_TC8234 */
