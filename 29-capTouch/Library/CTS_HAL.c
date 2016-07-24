/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
 /*!
 *  @file   CTS_HAL.c
 *
 *  @brief  This file contains the source for the different implementations.
 *
 *  @par    Project:
 *             MSP430 Capacitive Touch Library 
 *
 *  @par    Developed using:
 *             CCS Version : 5.2.1.00018, w/support for GCC extensions (--gcc)
 *  /n         IAR Version : 5.51.2 [Kickstart]
 *
 *  @author  C. Sterzik
 *  @author  T. Hwang
 *  @author  E. Fu
 *
 *  @version     1.4 Addresses Bux fixes in v1.3 with new devices.
 *
 *  @par    Supported Hardware Implementations:
 *              - TI_CTS_RO_COMPAp_TA0_WDTp_HAL()
 *              - TI_CTS_fRO_COMPAp_TA0_SW_HAL()
 *              - TI_CTS_fRO_COMPAp_SW_TA0_HAL()
 *              - TI_CTS_RO_COMPAp_TA1_WDTp_HAL()
 *              - TI_CTS_fRO_COMPAp_TA1_SW_HAL()
 *              - TI_CTS_RC_PAIR_TA0_HAL()
 *              - TI_CTS_RO_PINOSC_TA0_WDTp_HAL()
 *              - TI_CTS_RO_PINOSC_TA0_HAL()
 *              - TI_CTS_fRO_PINOSC_TA0_SW_HAL()
 *              - TI_CTS_RO_COMPB_TA0_WDTA_HAL()
 *              - TI_CTS_RO_COMPB_TA1_WDTA_HAL()
 *              - TI_CTS_fRO_COMPB_TA0_SW_HAL()
 *              - TI_CTS_fRO_COMPB_TA1_SW_HAL()
 *           /n  (1.1)
 *              - TI_CTS_fRO_PINOSC_TA0_TA1_HAL()
 *              - TI_CTS_RO_PINOSC_TA0_TA1_HAL()
 *              - TI_CTS_RO_CSIO_TA2_WDTA_HAL()
 *              - TI_CTS_RO_CSIO_TA2_TA3_HAL()
 *              - TI_CTS_fRO_CSIO_TA2_TA3_HAL()
 *              - TI_CTS_RO_COMPB_TB0_WDTA_HAL()
 *              - TI_CTS_RO_COMPB_TA1_TA0_HAL()
 *              - TI_CTS_fRO_COMPB_TA1_TA0_HAL()
 *           /n  (1.2)
 *              - TI_CTS_RO_PINOSC_TB0_WDTA_HAL()
 *              - TI_CTS_RO_PINOSC_TA1_TB0_HAL()
 *              - TI_CTS_fRO_PINOSC_TA1_TA0_HAL()
 *              - TI_CTS_fRO_PINOSC_TA1_TB0_HAL()
 *           /n  (1.3)                                        
 *              - TI_CTS_RO_CSIO_TA0_WDTA_HAL() 
 *              - TI_CTS_RO_CSIO_TA0_TA1_HAL()
 *              - TI_CTS_RO_CSIO_TA0_RTC_HAL()
 *              - TI_CTS_fRO_CSIO_TA0_TA1_HAL()
 *              - TI_CTS_fRO_CSIO_TA0_SW_HAL()
 *
 */

/*! 
 *  @defgroup CTS_HAL Capacitive Touch Implementations
 *  @{
 */

#include "CTS_HAL.h"

#ifdef RO_COMPB_TB0_WDTA
/*!
 *  ======== TI_CTS_RO_COMPB_TB0_WDTA_HAL ========
 *  @brief   RO method capactiance measurement using CompB, TimerB0, and WDTA
 *
 *     \n   Schematic Description of CompB forming relaxation oscillator and
 *          coupling (connection) between the relaxation oscillator and TimerA0.
 *     \n      <- Output
 *     \n      -> Input
 *     \n      R  Resistor (typically 100Kohms)
 * 
 *     \n      element---R----<-CBOUT/TB0CLK                               
 * 
 *     \n   The WDTA interval represents the measurement window.  The number of 
 *          counts within the TB0R that have accumulated during the measurement
 *          window represents the capacitance of the element.
 * 
 *  @param group  pointer to the sensor to be measured
 *  @param counts pointer to where the measurements are to be written
 *  @return none
 */
void TI_CTS_RO_COMPB_TB0_WDTA_HAL(const struct Sensor *group,uint16_t *counts)
{ 
   uint8_t i;
   /*
    *  Allocate Context Save Variables
    *  Status Register: GIE bit only
    *  SFR: SFRIE1
    *  WDTA: WDTCTL
    *  TIMERB0: TB0CTL, TB0CCTL0, TB0CCR0
    *  COMPB: CBCTL0,CBCTL1,CBCTL2, CBCTL3
    */
    uint8_t contextSaveSR; 
    uint16_t contextSaveSFRIE1;
    uint16_t contextSaveWDTCTL;
    uint16_t contextSaveTB0CTL,contextSaveTB0CCTL0,contextSaveTB0CCR0;
    uint16_t contextSaveCBCTL0,contextSaveCBCTL1;
    uint16_t contextSaveCBCTL2,contextSaveCBCTL3;
    uint8_t contextSaveCboutDir,contextSaveCboutSel;  
    /* Perform context save of registers used. */
    contextSaveSR = __get_SR_register();
    contextSaveSFRIE1 = SFRIE1;
    contextSaveWDTCTL = WDTCTL;
    contextSaveWDTCTL &= 0x00FF;
    contextSaveWDTCTL |= WDTPW;        
    contextSaveTB0CTL = TB0CTL;
    contextSaveTB0CCTL0 = TB0CCTL0;
    contextSaveTB0CCR0 = TB0CCR0;
    contextSaveCBCTL0 = CBCTL0;
    contextSaveCBCTL1 = CBCTL1;
    contextSaveCBCTL2 = CBCTL2;
    contextSaveCBCTL3 = CBCTL3;
	/* TAx naming convention is left to preserve compatibility. */
    contextSaveCboutDir = *(group->cboutTAxDirRegister);   
    contextSaveCboutSel = *(group->cboutTAxSelRegister);  
    /*
     *  Connect CBOUT with TB0. This also enables the feedback path for the
     *  Relaxation oscillator.
     */
    *(group->cboutTAxDirRegister) |= (group->cboutTAxBits);
    *(group->cboutTAxSelRegister) |= (group->cboutTAxBits);
    /*
     *  The COMPB reference is set to Vcc and the reference resistor taps are
     *  Vcc*(0x18+1)/32 for CBOUT = 1 and Vcc*((0x04+1)/32 for CBOUT = 0.
     *  If Vcc is 3.0V, then the Vih is 2.34V and the Vil is 0.47V.  In the
     *  event that CBOUT is connected to DVIO which is not equal to Vcc, then
     *  these voltage levels need to be adjusted.
     */
    CBCTL2 = CBRS_1 + CBREF14 + CBREF13 + CBREF02;
    CBCTL3 |= (group->cbpdBits);         // set CPD bits to disable digital IO
    /*
     *  TimerB0 is the measurement timer and counts the number of relaxation
     *  oscillation cycles of the element which is connected to TBCLK.
     *  TimerB0 is in continuous mode.  TB0CCR0 is configured as a capture
     *  register and will be triggered as a SW capture event.
     */
    TB0CTL = TBSSEL_0+MC_2;
    TB0CCTL0 = CM_3+CCIS_2+CAP+SCS;
    /*
     *  The WDTA is the gate (measurement interval) timer.  The number of
     *  oscillations counted, by TimerB0, within the gate interval represents
     *  the measured capacitance.
     */
    SFRIE1 |= WDTIE;        // Enable WDTA interrupt
    CBCTL1 = CBON;          // Turn on COMPB w/out filter
    for (i = 0; i<(group->numElements); i++)
    {
        /* Turn on specific comparator input. */
        CBCTL0 = CBIMEN + (group->arrayPtr[i])->inputBits;
        TB0CTL |= TBCLR;                   // Clear TimerB0, measurement timer
        TB0CTL &= ~TBIFG;                  // Clear overflow flag
        /*
         *  The measGateSource represents the gate source for the WDTA, which
         *  can be sourced from ACLK, SMCLK, VLOCLK or X_CLK. The
         *  accumulationCycles represents the watchdog timer interval select.
         */
        WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+ group->measGateSource
                + group->accumulationCycles;
        /*
         *  The interrupt handler is defined in WDT_VECTOR, which simply clears
         *  the low power mode bits in the Status Register before returning
         *  from the ISR.
         */
        if(group->measGateSource == GATE_WDT_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE); //Enable the GIE and wait for ISR
        }
        else
        {
	        __bis_SR_register(LPM0_bits+GIE);
        }
        TB0CCTL0 ^= CCIS0;  // Create SW capture of TB0R into TB0CCR0.
        WDTCTL = WDTPW + WDTHOLD;  // Halt watchdog timer
        if(TB0CTL & TBIFG)
        {
            /*
             *  If a rollover in the timer has occurred then set counts to
             *  0.  This will prevent erroneous data from entering the baseline
             *  tracking algorithm.
             */
            counts[i] = 0;
        }
        else
        {
            counts[i] = TB0CCR0;  // Save result
        }
    }  // End For Loop
    /* Context restore GIE within Status Register and registers used. */
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);
    }
    SFRIE1 = contextSaveSFRIE1;
    WDTCTL = contextSaveWDTCTL;
    TB0CTL = contextSaveTB0CTL;
    TB0CCTL0 = contextSaveTB0CCTL0;
    TB0CCR0 = contextSaveTB0CCR0;
    CBCTL0 = contextSaveCBCTL0;
    CBCTL1 = contextSaveCBCTL1;
    CBCTL2 = contextSaveCBCTL2;
    CBCTL3 = contextSaveCBCTL3;
    *(group->cboutTAxDirRegister) = contextSaveCboutDir;
    *(group->cboutTAxSelRegister) = contextSaveCboutSel;  
}
#endif

#ifdef fRO_CSIO_TA2_TA3 
/*!
 *  ======== TI_CTS_RO_CSIO_TA2_TA3_HAL ========
 *  @brief  fRO using Capacitive Touch IO, TimerA2, and TimerA3
 *          
 *  \n      Schematic Description: 
 * 
 *  \n      element-----+->Px.y
 * 
 *  \n      The TimerA2 interval represents the measurement window.  The number
 *          of counts within the TA3R that have accumulated during the
 *          measurement window represents the capacitance of the element.
 * 
 *  @param group  pointer to the sensor to be measured
 *  @param counts pointer to where the measurements are to be written
 *  @return none
 */
void TI_CTS_fRO_CSIO_TA2_TA3_HAL(const struct Sensor *group,uint16_t *counts)
{ 
    uint8_t i;
    /*
     *  Allocate Context Save Variables
     *  Status Register: GIE bit only
     *  SFR: SFRIE1
     *  TIMERA2: TA2CTL, TA2CCTL0, TA2CCR0
     *  TIMERA3: TA3CTL, TA3CCTL0, TA3CCR0
     *  CSIO: CSIOxCTL
     */
    uint8_t contextSaveSR; 
    uint16_t contextSaveTA2CTL,contextSaveTA2CCTL0,contextSaveTA2CCR0;
    uint16_t contextSaveTA3CTL,contextSaveTA3CCTL0,contextSaveTA3CCR0;
    uint8_t contextSaveCtl;
    /* Perform context save of registers used. */
    contextSaveSR = __get_SR_register();
    contextSaveTA2CTL = TA2CTL;
    contextSaveTA2CCTL0 = TA2CCTL0;
    contextSaveTA2CCR0 = TA2CCR0;
    contextSaveTA3CTL = TA3CTL;
    contextSaveTA3CCTL0 = TA3CCTL0;
    contextSaveTA3CCR0 = TA3CCR0;
    contextSaveCtl = *(group->inputCapsioctlRegister);
    /*
     *  TimerA3 is the measurement timer and counts the number of clock cycles
     *  of the source which is connected to measGateSource, typically SMCLK.
     *  TimerA3 is in continuous mode.  TA3CCR0 is configured as a capture
     *  register and will be triggered as a SW capture event.
     */
    TA3CTL = group->measGateSource + MC_2;
    TA3CCTL0 = CM_3+CCIS_2+CAP+SCS;
    /*
     *  TimerA2 is the gate (measurement interval) timer.  The number of
     *  oscillations counted, by TimerA3, within the gate interval represents
     *  the measured capacitance.  The input is TA2CLK.
     */
    TA2CCR0 = (group->accumulationCycles);
    TA2CTL = TASSEL_3+group->sourceScale;
    TA2CCTL0 = CCIE;
    for (i = 0; i<(group->numElements); i++)
    {
    	/* Enable Capacitive Touch IO oscillation */
	    *(group->inputCapsioctlRegister)
	            = ((group->arrayPtr[i])->inputBits)+CAPSIOEN;
        TA3CTL |= TACLR;                   // Clear TimerA3, measurement timer
        TA3CTL &= ~TAIFG;                  // Clear overflow flag
        TA2CTL |= (TACLR + MC_1);          // Clear and start TimerA3
        /*
		 *  The measGateSource represents the measurement source for timer
		 *  TIMERA3, which can be sourced from TACLK, ACLK, SMCLK, or INCLK.
		 *  The interrupt handler is defined in TIMER2_A0_VECTOR, which simply
		 *  clears the low power mode bits in the Status Register before
		 *  returning from the ISR.
		 */
        if(group->measGateSource == TIMER_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE);  // Enable GIE and wait for ISR
        }
        else
        {
            __bis_SR_register(LPM0_bits+GIE);
        }
        TA3CCTL0 ^= CCIS0;  // Create SW capture of TA3R into TA3CCR0
        TA2CTL &= ~MC_1;    // Halt Timer
        if(TA3CTL & TAIFG)
        {
        	/*
			 *  If a rollover in the timer has occurred then set counts to
			 *  0.  This will prevent erroneous data from entering the baseline
			 *  tracking algorithm.
			 */
			counts[i] = 0;
        }
        else
        {
            counts[i] = TA3CCR0;  // Save result
        }
    } // End For Loop
    /* Context restore GIE within Status Register and registers used. */
    *(group->inputCapsioctlRegister) = contextSaveCtl;
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);
    }
    TA2CTL = contextSaveTA2CTL;
    TA2CCTL0 = contextSaveTA2CCTL0;
    TA2CCR0 = contextSaveTA2CCR0;
    TA3CTL = contextSaveTA3CTL;
    TA3CCTL0 = contextSaveTA3CCTL0;
    TA3CCR0 = contextSaveTA3CCR0;
}
#endif

#ifdef RO_CSIO_TA2_TA3 
/*!
 *  ======== TI_CTS_RO_CSIO_TA2_TA3_HAL ========
 *  @brief   RO method using Capacitive Touch IO, TimerA2, and TimerA3
 *
 *  \n      Schematic Description: 
 *  \n      element-----+->Px.y
 * 
 *  \n      The TimerA3 interval represents the measurement window.  The number 
 *          of counts within the TA2R that have accumulated during the 
 *          measurement window represents the capacitance of the element.
 * 
 *  @param group  pointer to the sensor to be measured
 *  @param counts pointer to where the measurements are to be written
 *  @return        none
 */
void TI_CTS_RO_CSIO_TA2_TA3_HAL(const struct Sensor *group,uint16_t *counts)
{ 
    uint8_t i;
    /*
     *  Allocate Context Save Variables
     *  Status Register: GIE bit only
     *  SFR: SFRIE1
     *  TIMERA2: TA2CTL, TA2CCTL0, TA2CCR0
     *  TIMERA3: TA3CTL, TA3CCTL0, TA3CCR0
     *  CSIO: CSIOxCTL
     */
    uint8_t contextSaveSR; 
    uint16_t contextSaveTA2CTL,contextSaveTA2CCTL0,contextSaveTA2CCR0;
    uint16_t contextSaveTA3CTL,contextSaveTA3CCTL0,contextSaveTA3CCR0;
    uint8_t contextSaveCtl;
    /* Perform context save of registers used. */
    contextSaveSR = __get_SR_register();
    contextSaveTA2CTL = TA2CTL;
    contextSaveTA2CCTL0 = TA2CCTL0;
    contextSaveTA2CCR0 = TA2CCR0;
    contextSaveTA3CTL = TA3CTL;
    contextSaveTA3CCTL0 = TA3CCTL0;
    contextSaveTA3CCR0 = TA3CCR0;
    contextSaveCtl = *(group->inputCapsioctlRegister);
    /*
     *  TimerA2 is the measurement timer and counts the number of relaxation
     *  oscillation cycles of the element which is connected to TA2CLK.
     *  TimerA2 is in continuous mode.  TA2CCR0 is configured as a capture
     *  register and will be triggered as a SW capture event.
     */
    TA2CTL = TASSEL_3+MC_2;
    TA2CCTL0 = CM_3+CCIS_2+CAP+SCS;
    /*
     *  TimerA3 is the gate (measurement interval) timer.  The number of
     *  oscillations counted, by TimerA2, within the gate interval represents
     *  the measured capacitance.
     */
    TA3CCR0 = (group->accumulationCycles);
    TA3CTL = group->measGateSource + group->sourceScale;
    TA3CCTL0 = CCIE;
    for (i = 0; i<(group->numElements); i++)
    {
    	/* Enable Capacitive Touch IO oscillation */
	    *(group->inputCapsioctlRegister)
	            = ((group->arrayPtr[i])->inputBits)+CAPSIOEN;
        TA2CTL |= TACLR;                   // Clear TimerA2, measurement timer
        TA2CTL &= ~TAIFG;                  // Clear overflow flag
        TA3CTL |= (TACLR + MC_1);          // Clear and start TimerA3
        /*
		 *  The measGateSource represents the gate source for timer TIMERA3,
		 *  which can be sourced from TACLK, ACLK, SMCLK, or INCLK.  The
		 *  interrupt handler is defined in TIMER3_A0_VECTOR, which simply
		 *  clears the low power mode bits in the Status Register before
		 *  returning from the ISR.
		 */
        if(group->measGateSource == TIMER_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE);  // Enable GIE and wait for ISR
        }
        else
        {
            __bis_SR_register(LPM0_bits+GIE);
        }
        TA2CCTL0 ^= CCIS0;  // Create SW capture of TA2R into TA2CCR0
        TA3CTL &= ~MC_1;    // Halt Timer
        if(TA2CTL & TAIFG)
        {
        	/*
			 *  If a rollover in the timer has occurred then set counts to
			 *  0.  This will prevent erroneous data from entering the baseline
			 *  tracking algorithm.
			 */
			counts[i] = 0;
        }
        else
        {
            counts[i] = TA2CCR0;  // Save result
        }
    } // End For Loop
    /* Context restore GIE within Status Register and registers used. */
    *(group->inputCapsioctlRegister) = contextSaveCtl;
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);
    }
    TA2CTL = contextSaveTA2CTL;
    TA2CCTL0 = contextSaveTA2CCTL0;
    TA2CCR0 = contextSaveTA2CCR0;
    TA3CTL = contextSaveTA3CTL;
    TA3CCTL0 = contextSaveTA3CCTL0;
    TA3CCR0 = contextSaveTA3CCR0;
}
#endif

#ifdef RO_CSIO_TA2_WDTA 
/*!
 *  ======== TI_CTS_RO_CSIO_TA2_WDTA_HAL ========
 *  @brief  RO method measurement using Capacitive Touch IO, TimerA2, and WDTA
 *
 *  \n      Schematic Description: 
 * 
 *  \n      element-----+->Px.y
 * 
 *  \n      The WDTA interval represents the measurement window.  The number of 
 *          counts within the TA2R that have accumulated during the measurement
 *          window represents the capacitance of the element.
 * 
 *  @param group  pointer to the sensor to be measured
 *  @param counts pointer to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_RO_CSIO_TA2_WDTA_HAL(const struct Sensor *group,uint16_t *counts)
{ 
    uint8_t i;
    /*
     *  Allocate Context Save Variables
     *  Status Register: GIE bit only
     *  SFR: SFRIE1
     *  WDT: WDTCTL
     *  TIMERA2: TA2CTL, TA2CCTL0, TA2CCR0
     *  CSIO: CSIOxCTL
     */
    uint8_t contextSaveSR; 
    uint8_t contextSaveSFRIE1;
    uint16_t contextSaveWDTCTL;
    uint16_t contextSaveTA2CTL,contextSaveTA2CCTL0,contextSaveTA2CCR0;
    uint8_t contextSaveCtl;
    /* Perform context save of registers used. */
    contextSaveSR = __get_SR_register();
    contextSaveSFRIE1 = SFRIE1;
    contextSaveWDTCTL = WDTCTL;
    contextSaveWDTCTL &= 0x00FF;
    contextSaveWDTCTL |= WDTPW;        
    contextSaveTA2CTL = TA2CTL;
    contextSaveTA2CCTL0 = TA2CCTL0;
    contextSaveTA2CCR0 = TA2CCR0;
    contextSaveCtl = *(group->inputCapsioctlRegister);
    /*
     *  TimerA2 is the measurement timer and counts the number of relaxation
     *  oscillation cycles of the element which is connected to TA2CLK.
     *  TimerA2 is in continuous mode.  TA2CCR0 is configured as a capture
     *  register and will be triggered as a SW capture event.
     */
    TA2CTL = TASSEL_3+MC_2;
    TA2CCTL0 = CM_3+CCIS_2+CAP+SCS;
    /*
     *  The WDTA is the gate (measurement interval) timer.  The number of
     *  oscillations counted, by TimerA2, within the gate interval represents
     *  the measured capacitance.
     */
    SFRIE1 |= WDTIE;        // Enable WDTA interrupt
    for (i = 0; i<(group->numElements); i++)
    {
	    /* Enable Capacitive Touch IO oscillation */
	    *(group->inputCapsioctlRegister)
	            = ((group->arrayPtr[i])->inputBits)+CAPSIOEN;
	    TA2CTL |= TACLR;        // Clear Timer_A2 measurement timer
	    TA2CTL &= ~TAIFG;       // Clear the overflow flag
        /*
         *  The measGateSource represents the gate source for the WDTA, which
         *  can be sourced from ACLK, SMCLK, VLOCLK or X_CLK. The
         *  accumulationCycles represents the watchdog timer interval select.
         */
        WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+ group->measGateSource
                + group->accumulationCycles;
        /*
         *  The interrupt handler is defined in WDT_VECTOR, which simply clears
         *  the low power mode bits in the Status Register before returning
         *  from the ISR.
         */
        if(group->measGateSource == GATE_WDT_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE); //Enable the GIE and wait for ISR
        }
        else
        {
	        __bis_SR_register(LPM0_bits+GIE);
        }
	    TA2CCTL0 ^= CCIS0;  // Create SW capture of TA2R into TA2CCR0
	    WDTCTL = WDTPW + WDTHOLD;  // Halt watchdog timer
        if(TA2CTL & TAIFG)
        {
        	/*
			 *  If a rollover in the timer has occurred then set counts to
			 *  0.  This will prevent erroneous data from entering the baseline
			 *  tracking algorithm.
			 */
			counts[i] = 0;
        }
        else
        {
            counts[i] = TA2CCR0;  // Save result
        }
    }  // End For Loop
    /* Context restore GIE within Status Register and registers used. */
    *(group->inputCapsioctlRegister) = contextSaveCtl;
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);
    }
    SFRIE1 = contextSaveSFRIE1;
    WDTCTL = contextSaveWDTCTL;
    TA2CTL = contextSaveTA2CTL;
    TA2CCTL0 = contextSaveTA2CCTL0;
    TA2CCR0 = contextSaveTA2CCR0;
}
#endif

#ifdef RO_CSIO_TA0_WDTA
/*!
 *  ======== TI_CTS_RO_CSIO_TA0_WDTA_HAL ========
 *  @brief  RO method measurement using Capacitive Touch IO, TimerA0, and WDTA
 *
 *  \n      Schematic Description:
 *
 *  \n      element-----+->Px.y
 *
 *  \n      The WDTA interval represents the measurement window.  The number of
 *          counts within the TA0R that have accumulated during the measurement
 *          window represents the capacitance of the element.
 *
 *  @param group  pointer to the sensor to be measured
 *  @param counts pointer to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_RO_CSIO_TA0_WDTA_HAL(const struct Sensor *group,uint16_t *counts)
{
    uint8_t i;
    /*
     *  Allocate Context Save Variables
     *  Status Register: GIE bit only
     *  SFR: SFRIE1
     *  WDT: WDTCTL
     *  TIMERA0: TA0CTL, TA0CCTL0, TA0CCR0
     *  CSIO: CSIOxCTL
     */
    uint8_t contextSaveSR;
    uint8_t contextSaveSFRIE1;
    uint16_t contextSaveWDTCTL;
    uint16_t contextSaveTA0CTL,contextSaveTA0CCTL0,contextSaveTA0CCR0;
    uint8_t contextSaveCtl;
    /* Perform context save of registers used. */
    contextSaveSR = __get_SR_register();
    contextSaveSFRIE1 = SFRIE1;
    contextSaveWDTCTL = WDTCTL;
    contextSaveWDTCTL &= 0x00FF;
    contextSaveWDTCTL |= WDTPW;
    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCTL0 = TA0CCTL0;
    contextSaveTA0CCR0 = TA0CCR0;
    contextSaveCtl = *(group->inputCapsioctlRegister);
    /*
     *  TimerA0 is the measurement timer and counts the number of relaxation
     *  oscillation cycles of the element which is connected to TA0CLK.
     *  TimerA0 is in continuous mode.  TA0CCR0 is configured as a capture
     *  register and will be triggered as a SW capture event.
     */
    TA0CTL = TASSEL_3+MC_2;
    TA0CCTL0 = CM_3+CCIS_2+CAP+SCS;
    /*
     *  The WDTA is the gate (measurement interval) timer.  The number of
     *  oscillations counted, by TimerA0, within the gate interval represents
     *  the measured capacitance.
     */
    SFRIE1 |= WDTIE;        // Enable WDTA interrupt
    for (i = 0; i<(group->numElements); i++)
    {
	    /* Enable Capacitive Touch IO oscillation */
	    *(group->inputCapsioctlRegister)
	            = ((group->arrayPtr[i])->inputBits)+CAPSIOEN;
	    TA0CTL |= TACLR;        // Clear Timer_A0 measurement timer
	    TA0CTL &= ~TAIFG;       // Clear the overflow flag
        /*
         *  The measGateSource represents the gate source for the WDTA, which
         *  can be sourced from ACLK, SMCLK, VLOCLK or X_CLK. The
         *  accumulationCycles represents the watchdog timer interval select.
         */
        WDTCTL = WDTPW+WDTTMSEL+WDTCNTCL+ group->measGateSource
                + group->accumulationCycles;
        /*
         *  The interrupt handler is defined in WDT_VECTOR, which simply clears
         *  the low power mode bits in the Status Register before returning
         *  from the ISR.
         */
        if(group->measGateSource == GATE_WDTA_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE); //Enable the GIE and wait for ISR
        }
        else
        {
	        __bis_SR_register(LPM0_bits+GIE);
        }
	    TA0CCTL0 ^= CCIS0;  // Create SW capture of TA0R into TA0CCR0
	    WDTCTL = WDTPW + WDTHOLD;  // Halt watchdog timer
        if(TA0CTL & TAIFG)
        {
        	/*
			 *  If a rollover in the timer has occurred then set counts to
			 *  0.  This will prevent erroneous data from entering the baseline
			 *  tracking algorithm.
			 */
			counts[i] = 0;
        }
        else
        {
            counts[i] = TA0CCR0;  // Save result
        }
    }  // End For Loop
    /* Context restore GIE within Status Register and registers used. */
    *(group->inputCapsioctlRegister) = contextSaveCtl;
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);
    }
    SFRIE1 = contextSaveSFRIE1;
    WDTCTL = contextSaveWDTCTL;
    TA0CTL = contextSaveTA0CTL;
    TA0CCTL0 = contextSaveTA0CCTL0;
    TA0CCR0 = contextSaveTA0CCR0;
}
#endif

#ifdef fRO_PINOSC_TA0_TA1 
/*!
 *  @brief   fRO method capacitance measurement using PinOsc IO, TimerA0, and
 *          TimerA1
 *
 *  \n       Schematic Description: 
 * 
 *  \n       element-----+->Px.y
 * 
 *  \n       The TimerA0 interval represents the measurement window.  The number 
 *           of counts within the TA1R that have accumulated during the 
 *           measurement window represents the capacitance of the element.
 * 
 *  @param   group Pointer to the structure describing the Sensor to be measured
 *  @param   counts Pointer to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_fRO_PINOSC_TA0_TA1_HAL(const struct Sensor *group,uint16_t *counts)
{ 
    uint8_t i;

    /*
     *  Context Save
     *  Status Register: GIE
     *  TIMERA0: TA0CTL, TA0CCTL0, TA0CCR0
     *  TIMERA1: TA1CTL, TA1CCTL0, TA1CCR1
     *  Ports: PxSEL, PxSEL2
     */
    uint8_t contextSaveSR; 
    uint16_t contextSaveTA0CTL,contextSaveTA0CCTL0,contextSaveTA0CCR0;
    uint16_t contextSaveTA1CTL,contextSaveTA1CCTL0,contextSaveTA1CCR0;
    uint8_t contextSaveSel,contextSaveSel2;

    contextSaveSR = __get_SR_register();
    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCTL0 = TA0CCTL0;
    contextSaveTA0CCR0 = TA0CCR0;
    contextSaveTA1CTL = TA1CTL;
    contextSaveTA1CCTL0 = TA1CCTL0;
    contextSaveTA1CCR0 = TA1CCR0;
    
//** Setup Measurement timer***************************************************
// Choices are TA0,TA1,TB0,TB1,TD0,TD1 these choices are pushed up into the 
// capacitive touch layer.
 
    // Configure Measurement interval with TimerA0
    TA0CCR0 = (group->accumulationCycles);
    /*
     *  INCLK, IDx settings from sourceScale definition
     */
    TA0CTL = TASSEL_3 + group->sourceScale;
    TA0CCTL0 = CCIE;
    
    // Configure and start measurment timerA1
    TA1CTL = group->measGateSource + MC_2 + TACLR;  // cont
    TA1CCTL0 = CM_3+CCIS_2+CAP+SCS;       // Pos&Neg,GND,Cap, Sync
    
    for (i = 0; i<(group->numElements); i++)
    {
        // Context Save
        contextSaveSel = *((group->arrayPtr[i])->inputPxselRegister);
        contextSaveSel2 = *((group->arrayPtr[i])->inputPxsel2Register);
        // Configure Ports for relaxation oscillator
        *((group->arrayPtr[i])->inputPxselRegister) &= ~((group->arrayPtr[i])->inputBits);
        *((group->arrayPtr[i])->inputPxsel2Register) |= ((group->arrayPtr[i])->inputBits);

        TA1CTL |= TACLR;
    	TA1CTL &= ~TAIFG;
        TA0CTL |= (TACLR + MC_1);  // Clear Timer, Up mode
        /*
         *  In this configuration measGateSource represents the measurement
         *  source for timer TIMERA1, which can be sourced from TACLK, ACLK,
         *  SMCLK, or INCLK.
         */
        if(group->measGateSource == TIMER_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE);
        }
        else
        {
            __bis_SR_register(LPM0_bits+GIE);
        }
        TA1CCTL0 ^= CCIS0;        // Create SW capture of CCR1
        TA0CTL &= ~MC_1;          // Halt Timer
        if(TA1CTL & TAIFG)
        {
            /*
             *  If a rollover in the timer has occurred then set counts to
             *  0.  This will prevent erroneous data from entering the baseline
             *  tracking algorithm.
             */
        	counts[i] = 0;
        }
        else
        {
            counts[i] = TA1CCR0;      // Save result
        }
        // Context Restore
        *((group->arrayPtr[i])->inputPxselRegister) = contextSaveSel;
        *((group->arrayPtr[i])->inputPxsel2Register) = contextSaveSel2;
    } // End for loop
    // Context Restore
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);
    }
    TA0CTL = contextSaveTA0CTL;
    TA0CCTL0 = contextSaveTA0CCTL0;
    TA0CCR0 = contextSaveTA0CCR0;
    TA1CTL = contextSaveTA1CTL;
    TA1CCTL0 = contextSaveTA1CCTL0;
    TA1CCR0 = contextSaveTA1CCR0;
}
#endif

#ifdef RO_PINOSC_TA0_TA1 
/*!
 *  ======== TI_CTS_RO_PINOSC_TA0_TA1_HAL ========
 *  @brief   RO method capacitance measurement using PinOsc IO, TimerA0, and
 *           TimerA1
 *
 *  \n       Schematic Description:
 *
 *  \n       element-----+->Px.y
 * 
 *  \n       The TimerA1 interval represents the gate (measurement) time.  The
 *           number of oscillations that have accumulated in TA0R during the
 *           measurement time represents the capacitance of the element.
 * 
 *  @param group  pointer to the sensor to be measured
 *  @param counts pointer to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_RO_PINOSC_TA0_TA1_HAL(const struct Sensor *group,uint16_t *counts)
{ 
    uint8_t i;
    /*!
     *  Allocate Context Save Variables
     *  Status Register: GIE bit only
     *  TIMERA0: TA0CTL, TA0CCTL0, TA0CCR0
     *  TIMERA1: TA1CTL, TA1CCTL0, TA1CCR0
     *  Ports: PxSEL, PxSEL2
     */
    uint8_t contextSaveSR; 
    uint16_t contextSaveTA0CTL,contextSaveTA0CCTL0,contextSaveTA0CCR0;
    uint16_t contextSaveTA1CTL,contextSaveTA1CCTL0,contextSaveTA1CCR0;
    uint8_t contextSaveSel,contextSaveSel2;
    /*
     *  Perform context save of registers used except port registers which are
     *  saved and restored within the for loop as each element within the
     *  sensor is measured.
     */
    contextSaveSR = __get_SR_register();
    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCTL0 = TA0CCTL0;
    contextSaveTA0CCR0 = TA0CCR0;
    contextSaveTA1CTL = TA1CTL;
    contextSaveTA1CCTL0 = TA1CCTL0;
    contextSaveTA1CCR0 = TA1CCR0;
    /*
     *  TimerA0 is the measurement timer and counts the number of relaxation
     *  oscillation cycles of the electrode which is routed to INCLK.  TA0 is
     *  in continuous mode and sourced from INCLK.
     */
    TA0CTL = TASSEL_3+MC_2;
    TA0CCTL0 = CM_3+CCIS_2+CAP+SCS;            // Setup for SW capture
    /*
     *  TimerA1 is the gate (measurement interval) timer.  The number of
     *  oscillations counted within the gate interval represents the measured
     *  capacitance.
     */
    TA1CCR0 = (group->accumulationCycles);
    // Establish source and scale of timerA1, but halt the timer.
    TA1CTL = group->measGateSource + group->sourceScale;
    TA1CCTL0 = CCIE;  // Enable Interrupt when timer counts to TA1CCR0.
    for (i = 0; i<(group->numElements); i++)
    {
        // Context Save Port Registers
        contextSaveSel = *((group->arrayPtr[i])->inputPxselRegister);
        contextSaveSel2 = *((group->arrayPtr[i])->inputPxsel2Register);
        // Configure Ports for relaxation oscillator
        *((group->arrayPtr[i])->inputPxselRegister)
        		&= ~((group->arrayPtr[i])->inputBits);
        *((group->arrayPtr[i])->inputPxsel2Register)
        		|= ((group->arrayPtr[i])->inputBits);
        TA0CTL |= TACLR;
        TA0CTL &= ~TAIFG;
        TA1CTL |= (TACLR + MC_1);
        /*!
		 *  The measGateSource represents the gate source for timer TIMERA1,
		 *  which can be sourced from TACLK, ACLK, SMCLK, or INCLK.  The
		 *  interrupt handler is defined in TIMER1_A0_VECTOR, which simply
		 *  clears the low power mode bits in the Status Register before
		 *  returning from the ISR.
		 */
        if(group->measGateSource == TIMER_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE);  // Enable GIE and wait for ISR
        }
        else
        {
            __bis_SR_register(LPM0_bits+GIE);  // Enable GIE and wait for ISR
        }
        TA0CCTL0 ^= CCIS0;  // Create SW capture of TA0CCR into TA0CCR0.
        TA1CTL &= ~MC_1;    // Halt Timer
        if(TA0CTL & TAIFG)
        {
        	/*
			 *  If a rollover in the timer has occurred then set counts to
			 *  0.  This will prevent erroneous data from entering the baseline
			 *  tracking algorithm.
			 */
			counts[i] = 0;
        }
        else
        {
            counts[i] = TA0CCR0;  // Save result
        }
        // Context Restore Port Registers
        *((group->arrayPtr[i])->inputPxselRegister) = contextSaveSel;
        *((group->arrayPtr[i])->inputPxsel2Register) = contextSaveSel2;
    } // End for Loop
    /*
     *  Context restore GIE within Status Register and all timer registers
     *  used.
     */
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);
    }
    TA0CTL = contextSaveTA0CTL;
    TA0CCTL0 = contextSaveTA0CCTL0;
    TA0CCR0 = contextSaveTA0CCR0;
    TA1CTL = contextSaveTA1CTL;
    TA1CCTL0 = contextSaveTA1CCTL0;
    TA1CCR0 = contextSaveTA1CCR0;
}
#endif

#ifdef RO_COMPAp_TA0_WDTp
/*!
 *  @brief   RO method capactiance measurement using CompA+, TimerA0, and WDT+
 *
 *  \n       Schematic Description of CompA+ forming relaxation oscillator and
 *           coupling (connection) between the relaxation oscillator and 
 *           TimerA0.
 *  \n       <- Output
 *  \n       -> Input
 *  \n       R  Resistor (typically 100Kohms)
 *      
 *                       +-<-Px.y (reference)
 *                       |
 *                       R
 *                       |
 *                   +---+-->COMPA+
 *                   |   |
 *                   R   R                
 *                   |   |
 *                  GND  |
 *                       |                       
 *                       +-->TACLK
 *                       |
 *          element-+-R--+-<-CAOUT                               
 *                  |
 *                  +------->COMPA- 
 *    
 *  \n      The WDT+ interval represents the measurement window.  The number of 
 *          counts within the TA0R that have accumulated during the measurement
 *          window represents the capacitance of the element.
 * 
 *  @param   group Address of the structure describing the Sensor to be measured
 *  @param   counts Address to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_RO_COMPAp_TA0_WDTp_HAL(const struct Sensor *group, uint16_t *counts)
{ 
    uint8_t i;
//** Context Save
//  Status Register: 
//  WDTp: IE1, WDTCTL
//  TIMERA0: TACTL, TACCTL1
//  COMPAp: CACTL1, CACTL2, CAPD
//  Ports: caoutDIR, caoutSel, txclkDIR, txclkSel, caoutSel2, txclkSel2, refout, refdir 
    uint8_t contextSaveSR; 
    uint8_t contextSaveIE1;
    uint16_t contextSaveWDTCTL;
    uint16_t contextSaveTACTL,contextSaveTACCTL1,contextSaveTACCR1;
    uint8_t contextSaveCACTL1,contextSaveCACTL2,contextSaveCAPD;
    uint8_t contextSaveCaoutDir,contextSaveCaoutSel;  
    uint8_t contextSavetxclkDir,contextSavetxclkSel;    
    uint8_t contextSaveRefDir,contextSaveRefOutSel;  
    #ifdef SEL2REGISTER
    uint8_t contextSaveCaoutSel2,contextSaveTxclkSel2; 
    
    contextSaveCaoutSel2 = *(group->caoutSel2Register);
    contextSaveTxclkSel2 = *(group->txclkSel2Register);       
    #endif    
    contextSaveSR = __get_SR_register();
    contextSaveIE1 = IE1;
    contextSaveWDTCTL = WDTCTL;
    contextSaveWDTCTL &= 0x00FF;
    contextSaveWDTCTL |= WDTPW;        
    contextSaveTACTL = TACTL;
    contextSaveTACCTL1 = TACCTL1;
    contextSaveTACCR1 = TACCR1;
    contextSaveCACTL1 = CACTL1;
    contextSaveCACTL2 = CACTL2;
    contextSaveCAPD = CAPD;
    contextSaveCaoutDir = *(group->caoutDirRegister);
    contextSaveCaoutSel = *(group->caoutSelRegister);  
    contextSavetxclkDir = *(group->txclkDirRegister);
    contextSavetxclkSel = *(group->txclkSelRegister);    
    contextSaveRefDir = *(group->refPxdirRegister);
    contextSaveRefOutSel = *(group->refPxoutRegister);  
    
    TACTL = TASSEL_0+MC_2;                // TACLK, cont mode
    TACCTL1 = CM_3+CCIS_2+CAP+SCS;        // Pos&Neg,GND,Cap
	  
    *(group->caoutDirRegister) |= group->caoutBits;
    *(group->txclkDirRegister) &= ~group->txclkBits;
    *(group->caoutSelRegister) |= group->caoutBits;
    *(group->txclkSelRegister) |= group->txclkBits;
    
    #ifdef SEL2REGISTER
    *(group->caoutSel2Register) |= group->caoutBits;
    *(group->txclkSel2Register) |= group->txclkBits;
    #endif
	  
    *(group->refPxdirRegister) |= group->refBits;
    *(group->refPxoutRegister) |= group->refBits;
    CACTL1 |= CAON;                       // Turn on comparator
    CAPD |= (group->capdBits); 
    IE1 |= WDTIE;                         // enable WDT interrupt
	
    for (i = 0; i<(group->numElements); i++)
    {
        CACTL2= group->refCactl2Bits + (group->arrayPtr[i])->inputBits;
	//**  Setup Gate Timer *****************************************************
	// Set duration of sensor measurment
	WDTCTL = WDTPW+WDTTMSEL+ group->measGateSource + group->accumulationCycles; 
	TACTL |= TACLR;                     // Clear Timer_A TAR
    if(group->measGateSource == GATE_WDTp_ACLK)
    {
        __bis_SR_register(LPM3_bits+GIE);   // Wait for WDT interrupt
    }
    else
    {
	    __bis_SR_register(LPM0_bits+GIE);   // Wait for WDT interrupt
    }
	TACCTL1 ^= CCIS0;                   // Create SW capture of CCR1
	counts[i] = TACCR1;                 // Save result
	WDTCTL = WDTPW + WDTHOLD;           // Stop watchdog timer
    }
	  // End Sequence
      
    //** Context Restore
    //  WDTp: IE1, WDCTL
    //  TIMERA0: TACTL, TACCTL1
    //  COMPAp: CACTL1, CACTL2, CAPD
    //  Ports: caoutDIR, caoutSel, txclkDIR, txclkSel, caoutSel2, txclkSel2, refout, refdir  
    #ifdef SEL2REGISTER  
    *(group->caoutSel2Register) = contextSaveCaoutSel2;
    *(group->txclkSel2Register) = contextSaveTxclkSel2;       
    #endif    
    __bis_SR_register(contextSaveSR);   
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);   // Wait for WDT interrupt        
    }
    IE1 = contextSaveIE1;
    WDTCTL = contextSaveWDTCTL;
    TACTL = contextSaveTACTL;
    TACCTL1 = contextSaveTACCTL1;
    TACCR1 = contextSaveTACCR1;
    CACTL1 = contextSaveCACTL1;
    CACTL2 = contextSaveCACTL2;
    CAPD = contextSaveCAPD;
    *(group->caoutDirRegister) = contextSaveCaoutDir;
    *(group->caoutSelRegister) = contextSaveCaoutSel;  
    *(group->txclkDirRegister) = contextSavetxclkDir;
    *(group->txclkSelRegister) = contextSavetxclkSel;    
    *(group->refPxdirRegister) = contextSaveRefDir;
    *(group->refPxoutRegister) = contextSaveRefOutSel;  
}
#endif

#ifdef fRO_COMPAp_TA0_SW
/*!
 *  @brief  RO method capactiance measurement using CompA+, TimerA0, and SW loop
 *
 *  \n      Schematic Description of CompA+ forming relaxation oscillator.
 *  \n      <- Output
 *  \n      -> Input
 *  \n      R  Resistor (typically 100Kohms)
 *      
 *                       +-<-Px.y (reference)
 *                       |
 *                       R
 *                       |
 *                   +---+-->COMPA+
 *                   |   |
 *                   R   R                
 *                   |   |
 *                  GND  |
 *                       |  
 *                       +-->TACLK                     
 *                       |
 *          element-+-R--+-<-CAOUT                               
 *                  |
 *                 +------->COMPA- 
 *      
 *  \n      The timer counts to TA0CCR0 representing the measurement window. The
 *          number of counts within the SW loop that have accumulated during the
 *          measurement window represents the capacitance of the element.
 * 
 *  @param   group Address of the structure describing the Sensor to be measured
 *  @param   counts Address to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_fRO_COMPAp_TA0_SW_HAL(const struct Sensor *group, uint16_t *counts)
{ 
    uint8_t i;
    uint16_t j;
//** Context Save
//  Status Register: 
//  TIMERA0: TACTL, TACCTL0
//  COMPAp: CACTL1, CACTL2, CAPD
//  Ports: caoutDIR, caoutSel, txclkDIR, txclkSel, caoutSel2, txclkSel2, refout, refdir 
    uint16_t contextSaveTACTL,contextSaveTACCTL0,contextSaveTACCR0;
    uint8_t contextSaveCACTL1,contextSaveCACTL2,contextSaveCAPD;
    uint8_t contextSaveCaoutDir,contextSaveCaoutSel;  
    uint8_t contextSavetxclkDir,contextSavetxclkSel;    
    uint8_t contextSaveRefDir,contextSaveRefOutSel;  
    #ifdef SEL2REGISTER
    uint8_t contextSaveCaoutSel2,contextSaveTxclkSel2; 
    
    contextSaveCaoutSel2 = *(group->caoutSel2Register);
    contextSaveTxclkSel2 = *(group->txclkSel2Register);       
    #endif    
    contextSaveTACTL = TACTL;
    contextSaveTACCTL0 = TACCTL0;
    contextSaveTACCR0 = TACCR0;
    contextSaveCACTL1 = CACTL1;
    contextSaveCACTL2 = CACTL2;
    contextSaveCAPD = CAPD;
    contextSaveCaoutDir = *(group->caoutDirRegister);
    contextSaveCaoutSel = *(group->caoutSelRegister);  
    contextSavetxclkDir = *(group->txclkDirRegister);
    contextSavetxclkSel = *(group->txclkSelRegister);    
    contextSaveRefDir = *(group->refPxdirRegister);
    contextSaveRefOutSel = *(group->refPxoutRegister);      
    
//** Setup Measurement timer***************************************************
 
    // Configure Timer TA0
    TACCR0 =(group->accumulationCycles);
    TACCTL0 &= ~CAP;
    // setup connections between CAOUT and TA0
    *(group->caoutDirRegister) |= group->caoutBits;
    *(group->txclkDirRegister) &= ~group->txclkBits;
    *(group->caoutSelRegister) |= group->caoutBits;
    *(group->txclkSelRegister) |= group->txclkBits;
    
    #ifdef SEL2REGISTER
    *(group->caoutSel2Register) |= group->caoutBits;
    *(group->txclkSel2Register) |= group->txclkBits;
    #endif
    // setup reference
    *(group->refPxdirRegister) |= group->refBits;
    *(group->refPxoutRegister) |= group->refBits;
    CACTL1 |= CAON;                       // Turn on comparator
    CAPD |= (group->capdBits); 
    
    for (i = 0; i<(group->numElements); i++)
    {
       j=0;
       CACTL2= group->refCactl2Bits + (group->arrayPtr[i])->inputBits;
        //**  Setup Gate Timer **************
        // Set duration of sensor measurment
        TACTL = TASSEL_0+TACLR+MC_1;        // TACLK, reset, up mode
        TACTL &= ~TAIFG;                    // clear IFG
        while(!(TACTL & TAIFG))
        {
            j++;
        } // end accumulation
        counts[i] = j;   
    }
    // End Sequence
    //** Context Restore
    //  TIMERA0: TACTL, TACCTL1
    //  COMPAp: CACTL1, CACTL2, CAPD
    //  Ports: caoutDIR, caoutSel, txclkDIR, txclkSel, caoutSel2, txclkSel2, refout, refdir  
    #ifdef SEL2REGISTER  
    *(group->caoutSel2Register) = contextSaveCaoutSel2;
    *(group->txclkSel2Register) = contextSaveTxclkSel2;       
    #endif    
    TACTL = contextSaveTACTL;
    TACCTL0 = contextSaveTACCTL0;
    TACCR0 = contextSaveTACCR0;
    CACTL1 = contextSaveCACTL1;
    CACTL2 = contextSaveCACTL2;
    CAPD = contextSaveCAPD;
    *(group->caoutDirRegister) = contextSaveCaoutDir;
    *(group->caoutSelRegister) = contextSaveCaoutSel;  
    *(group->txclkDirRegister) = contextSavetxclkDir;
    *(group->txclkSelRegister) = contextSavetxclkSel;    
    *(group->refPxdirRegister) = contextSaveRefDir;
    *(group->refPxoutRegister) = contextSaveRefOutSel;  
}
#endif

#ifdef fRO_COMPAp_SW_TA0
/*!
 *  @brief   RO method capactiance measurement using CompA+, TimerA0, and SW loop
 *
 *  \n       Schematic Description of CompA+ forming relaxation oscillator.
 *  \n       <- Output
 *  \n       -> Input
 *  \n       R  Resistor (typically 100Kohms)
 *   
 *                        +-<-Px.y (reference)
 *                        |
 *                        R
 *                        |
 *                    +---+-->COMPA+
 *                    |   |
 *                    R   R                
 *                    |   |
 *                   GND  |
 *                        |  
 *                        |
 *           element-+-R--+-<-CAOUT                               
 *                   |
 *                   +------->COMPA- 
 *     
 *  \n       The SW loop counts to 'n' accumulationCycles, representing the 
 *           measurement window. The number of timer counts within TA0R register
 *           represents the capacitance of the element.
 * 
 *  @param   group Address of the structure describing the Sensor to be measured
 *  @param   counts Address to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_fRO_COMPAp_SW_TA0_HAL(const struct Sensor *group, uint16_t *counts)
{ 
    uint8_t i;
    uint16_t j;
    //** Context Save
//  Status Register: 
//  TIMERA0: TACTL, TACCTL0, TACCTL1
//  COMPAp: CACTL1, CACTL2, CAPD
//  Ports: caoutDIR, caoutSel, caoutSel2, refout, refdir 
    uint16_t contextSaveTACTL,contextSaveTACCTL0,contextSaveTACCTL1;
    uint16_t contextSaveTACCR0,contextSaveTACCR1;
    uint8_t contextSaveCACTL1,contextSaveCACTL2,contextSaveCAPD;
    uint8_t contextSaveCaoutDir,contextSaveCaoutSel;  
    uint8_t contextSaveRefDir,contextSaveRefOutSel;  
    #ifdef SEL2REGISTER
    uint8_t contextSaveCaoutSel2,contextSaveTxclkSel2; 
    
    contextSaveCaoutSel2 = *(group->caoutSel2Register);
    #endif    
    contextSaveTACTL = TACTL;
    contextSaveTACCTL0 = TACCTL0;
    contextSaveTACCTL1 = TACCTL1;
    contextSaveTACCR0 = TACCR0;
    contextSaveTACCR1 = TACCR1;
    contextSaveCACTL1 = CACTL1;
    contextSaveCACTL2 = CACTL2;
    contextSaveCAPD = CAPD;
    contextSaveCaoutDir = *(group->caoutDirRegister);
    contextSaveCaoutSel = *(group->caoutSelRegister);  
    contextSaveRefDir = *(group->refPxdirRegister);
    contextSaveRefOutSel = *(group->refPxoutRegister);      
//** Setup Measurement timer***************************************************
 
    // Configure Timer TA0
    TACCTL0 = CM_3+CCIS_2+CAP+SCS;            // Pos&Neg,GND,Cap
    TACCTL1 = CM_3+CCIS_2+CAP+SCS;            // Pos&Neg,GND,Cap

    // setup connections between CAOUT and TA0
    *(group->caoutDirRegister) |= group->caoutBits;
    *(group->caoutSelRegister) |= group->caoutBits;
    
    #ifdef SEL2REGISTER
    *(group->caoutSel2Register) |= group->caoutBits;
    #endif
    // setup reference
    *(group->refPxdirRegister) |= group->refBits;
    *(group->refPxoutRegister) |= group->refBits;
    CACTL1 |= CAON;                       // Turn on comparator
    CAPD |= (group->capdBits); 
    for (i = 0; i<(group->numElements); i++)
    {
        CACTL2= group->refCactl2Bits + (group->arrayPtr[i])->inputBits;
        //**  Setup Gate Timer **************
        // Set duration of sensor measurment
        TACTL = group->measGateSource+group->sourceScale+TACLR+MC_2;
        TACCTL0 ^= CCIS0;                     // Create SW capture of CCR0
        for(j = group->accumulationCycles; j > 0; j--)
        {
            CACTL1 &= ~CAIFG;
            while(!(CACTL1 & CAIFG));
        }
        TACCTL1 ^= CCIS0;                     // Create SW capture of CCR1
        counts[i] = TACCR1;               // Save result
        counts[i] -= TACCR0;               // Save result
        TACCTL0 &= ~CCIFG;
        TACCTL1 &= ~CCIFG;
    }
    // End Sequence
    //** Context Restore
    //  WDTp: IE1, WDCTL
    //  TIMERA0: TACTL, TACCTL0, TACCTL1, TACCR0, TACCR1
    //  COMPAp: CACTL1, CACTL2, CAPD
    //  Ports: caoutDIR, caoutSel, txclkDIR, caoutSel2, refout, refdir  
    #ifdef SEL2REGISTER  
    *(group->caoutSel2Register) = contextSaveCaoutSel2;
    #endif    
    TACTL = contextSaveTACTL;
    TACCTL0 = contextSaveTACCTL0;
    TACCTL1 = contextSaveTACCTL1;
    TACCR0 = contextSaveTACCR0;
    TACCR1 = contextSaveTACCR1;
    CACTL1 = contextSaveCACTL1;
    CACTL2 = contextSaveCACTL2;
    CAPD = contextSaveCAPD;
    *(group->caoutDirRegister) = contextSaveCaoutDir;
    *(group->caoutSelRegister) = contextSaveCaoutSel;  
    *(group->refPxdirRegister) = contextSaveRefDir;
    *(group->refPxoutRegister) = contextSaveRefOutSel;  
}
#endif

#ifdef RO_COMPAp_TA1_WDTp
/*!
 *  @brief   RO method capactiance measurement using CompA+, TimerA1, and WDT+
 *
 *  \n       Schematic Description of CompA+ forming relaxation oscillator and
 *           coupling (connection) between the relaxation oscillator and TimerA0.
 *  \n       <- Output
 *  \n       -> Input
 *  \n       R  Resistor (typically 100Kohms)
 *  
 *                        +-<-Px.y (reference)
 *                        |
 *                        R
 *                        |
 *                        +---+-->COMPA+
 *                        |   |
 *                        R   R                
 *                        |   |
 *                       GND  |
 *                            |                       
 *                            +-->TA1CLK
 *                            |
 *               element-+-R--+-<-CAOUT                               
 *                            |
 *                            +------->COMPA- 
 * 
 *  \n       The WDT+ interval represents the measurement window.  The number of 
 *           counts within the TA0R that have accumulated during the measurement
 *           window represents the capacitance of the element.
 * 
 *  @param   group Address of the structure describing the Sensor to be measured
 *  @param   counts Address to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_RO_COMPAp_TA1_WDTp_HAL(const struct Sensor *group, uint16_t *counts)
{ 
    uint8_t i;
//** Context Save
//  Status Register: 
//  WDTp: IE1, WDTCTL
//  TIMERA0: TA1CTL, TA1CCTL1
//  COMPAp: CACTL1, CACTL2, CAPD
//  Ports: caoutDIR, caoutSel, txclkDIR, txclkSel, caoutSel2, txclkSel2, refout, refdir 
    uint8_t contextSaveSR; 
    uint8_t contextSaveIE1;
    uint16_t contextSaveWDTCTL;
    uint16_t contextSaveTA1CTL,contextSaveTA1CCTL1;
    uint16_t contextSaveTA1CCR1;
    uint8_t contextSaveCACTL1,contextSaveCACTL2,contextSaveCAPD;
    uint8_t contextSaveCaoutDir,contextSaveCaoutSel;  
    uint8_t contextSavetxclkDir,contextSavetxclkSel;    
    uint8_t contextSaveRefDir,contextSaveRefOutSel;  
    #ifdef SEL2REGISTER
    uint8_t contextSaveCaoutSel2,contextSaveTxclkSel2; 
    
    contextSaveCaoutSel2 = *(group->caoutSel2Register);
    contextSaveTxclkSel2 = *(group->txclkSel2Register);       
    #endif    
    contextSaveSR = __get_SR_register();
    contextSaveIE1 = IE1;
    contextSaveWDTCTL = WDTCTL;
    contextSaveWDTCTL &= 0x00FF;
    contextSaveWDTCTL |= WDTPW;        
    contextSaveTA1CTL = TA1CTL;
    contextSaveTA1CCTL1 = TA1CCTL1;
    contextSaveTA1CCR1 = TA1CCR1;
    contextSaveCACTL1 = CACTL1;
    contextSaveCACTL2 = CACTL2;
    contextSaveCAPD = CAPD;
    contextSaveCaoutDir = *(group->caoutDirRegister);
    contextSaveCaoutSel = *(group->caoutSelRegister);  
    contextSavetxclkDir = *(group->txclkDirRegister);
    contextSavetxclkSel = *(group->txclkSelRegister);    
    contextSaveRefDir = *(group->refPxdirRegister);
    contextSaveRefOutSel = *(group->refPxoutRegister);  
//** Setup Measurement timer***************************************************
// Choices are TA0,TA1,TB0,TB1,TD0,TD1 these choices are pushed up into the 
// capacitive touch layer.
 
    TA1CTL = TASSEL_0+MC_2;                // TA1CLK, cont mode
    TA1CCTL1 = CM_3+CCIS_2+CAP+SCS;            // Pos&Neg,GND,Cap
      
    *(group->caoutDirRegister) |= group->caoutBits;
    *(group->txclkDirRegister) &= ~group->txclkBits;
    *(group->caoutSelRegister) |= group->caoutBits;
    *(group->txclkSelRegister) |= group->txclkBits;
    
    #ifdef SEL2REGISTER
    *(group->caoutSel2Register) |= group->caoutBits;
    *(group->txclkSel2Register) |= group->txclkBits;
    #endif
      
    *(group->refPxdirRegister) |= group->refBits;
    *(group->refPxoutRegister) |= group->refBits;
    CACTL1 |= CAON;                       // Turn on comparator
    CAPD |= (group->capdBits); 
    IE1 |= WDTIE;                         // enable WDT interrupt
    
    for (i = 0; i<(group->numElements); i++)
    {
       CACTL2= group->refCactl2Bits + (group->arrayPtr[i])->inputBits;
    //**  Setup Gate Timer *****************************************************
    // Set duration of sensor measurment
       WDTCTL = WDTPW+WDTTMSEL+ group->measGateSource + group->accumulationCycles; 
       TA1CTL |= TACLR;                     // Clear Timer_A TAR
       if(group->measGateSource == GATE_WDTp_ACLK)
       {
            __bis_SR_register(LPM3_bits+GIE);   // Wait for WDT interrupt
       }
       else
       {
	        __bis_SR_register(LPM0_bits+GIE);   // Wait for WDT interrupt
       }
       TA1CCTL1 ^= CCIS0;                   // Create SW capture of CCR1
       counts[i] = TA1CCR1;                 // Save result
       WDTCTL = WDTPW + WDTHOLD;           // Stop watchdog timer
    }
      // End Sequence
    //** Context Restore
    //  WDTp: IE1, WDCTL
    //  TIMERA0: TACTL, TACCTL1
    //  COMPAp: CACTL1, CACTL2, CAPD
    //  Ports: caoutDIR, caoutSel, txclkDIR, txclkSel, caoutSel2, txclkSel2, refout, refdir  
    #ifdef SEL2REGISTER  
    *(group->caoutSel2Register) = contextSaveCaoutSel2;
    *(group->txclkSel2Register) = contextSaveTxclkSel2;       
    #endif    
    __bis_SR_register(contextSaveSR);   
    if(!(contextSaveSR & GIE))
    {
      __bic_SR_register(GIE);   // Wait for WDT interrupt        
    }
    IE1 = contextSaveIE1;
    WDTCTL = contextSaveWDTCTL;
    TA1CTL = contextSaveTA1CTL;
    TA1CCTL1 = contextSaveTA1CCTL1;
    TA1CCR1 = contextSaveTA1CCR1;
    CACTL1 = contextSaveCACTL1;
    CACTL2 = contextSaveCACTL2;
    CAPD = contextSaveCAPD;
    *(group->caoutDirRegister) = contextSaveCaoutDir;
    *(group->caoutSelRegister) = contextSaveCaoutSel;  
    *(group->txclkDirRegister) = contextSavetxclkDir;
    *(group->txclkSelRegister) = contextSavetxclkSel;    
    *(group->refPxdirRegister) = contextSaveRefDir;
    *(group->refPxoutRegister) = contextSaveRefOutSel;  
}
#endif

#ifdef fRO_COMPAp_TA1_SW
/*!
 *  @brief   RO method capactiance measurement using CompA+, TimerA1, and SW loop
 *
 *           Schematic Description of CompA+ forming relaxation oscillator.
 *  \n       <- Output
 *  \n       -> Input
 *  \n       R  Resistor (typically 100Kohms)
 *   
 *                        +-<-Px.y (reference)
 *                        |
 *                        R
 *                        |
 *                    +---+-->COMPA+
 *                    |   |
 *                    R   R                
 *                    |   |
 *                   GND  |
 *                        |  
 *                        +-->TA1CLK                     
 *                        |
 *           element-+-R--+-<-CAOUT                               
 *                   |
 *                   +------->COMPA- 
 *    
 *  \n      The timer counts to TA1CCR0 representing the measurement window. The
 *          number of counts within the SW loop that have accumulated during the
 *          measurement window represents the capacitance of the element.
 * 
 *  @param   group Address of the structure describing the Sensor to be measured
 *  @param   counts Address to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_fRO_COMPAp_TA1_SW_HAL(const struct Sensor *group, uint16_t *counts)
{ 
    uint8_t i;
    uint16_t j;
    //** Context Save
//  Status Register: 
//  TIMERA0: TA1CTL, TA1CCTL0
//  COMPAp: CACTL1, CACTL2, CAPD
//  Ports: caoutDIR, caoutSel, txclkDIR, txclkSel, caoutSel2, txclkSel2, refout, refdir 
    uint16_t contextSaveTA1CTL,contextSaveTA1CCTL0,contextSaveTA1CCR0;
    uint8_t contextSaveCACTL1,contextSaveCACTL2,contextSaveCAPD;
    uint8_t contextSaveCaoutDir,contextSaveCaoutSel;  
    uint8_t contextSavetxclkDir,contextSavetxclkSel;    
    uint8_t contextSaveRefDir,contextSaveRefOutSel;  
    #ifdef SEL2REGISTER
    uint8_t contextSaveCaoutSel2,contextSaveTxclkSel2; 
    
    contextSaveCaoutSel2 = *(group->caoutSel2Register);
    contextSaveTxclkSel2 = *(group->txclkSel2Register);       
    #endif    
    contextSaveTA1CTL = TA1CTL;
    contextSaveTA1CCTL0 = TA1CCTL0;
    contextSaveTA1CCR0 = TA1CCR0;
    contextSaveCACTL1 = CACTL1;
    contextSaveCACTL2 = CACTL2;
    contextSaveCAPD = CAPD;
    contextSaveCaoutDir = *(group->caoutDirRegister);
    contextSaveCaoutSel = *(group->caoutSelRegister);  
    contextSavetxclkDir = *(group->txclkDirRegister);
    contextSavetxclkSel = *(group->txclkSelRegister);    
    contextSaveRefDir = *(group->refPxdirRegister);
    contextSaveRefOutSel = *(group->refPxoutRegister);      
//** Setup Measurement timer***************************************************
 
    // Configure Timer TA0
    TA1CCR0 =(group->accumulationCycles);
    // setup connections between CAOUT and TA0
    *(group->caoutDirRegister) |= group->caoutBits;
    *(group->txclkDirRegister) &= ~group->txclkBits;
    *(group->caoutSelRegister) |= group->caoutBits;
    *(group->txclkSelRegister) |= group->txclkBits;
    
    #ifdef SEL2REGISTER
    *(group->caoutSel2Register) |= group->caoutBits;
    *(group->txclkSel2Register) |= group->txclkBits;
    #endif
    // setup reference
    *(group->refPxdirRegister) |= group->refBits;
    *(group->refPxoutRegister) |= group->refBits;
    CACTL1 |= CAON;                       // Turn on comparator
    CAPD |= (group->capdBits); 
    
    for (i = 0; i<(group->numElements); i++)
    {
        j=0;
        CACTL2= group->refCactl2Bits + (group->arrayPtr[i])->inputBits;
        //**  Setup Gate Timer **************
        // Set duration of sensor measurment
        TA1CTL = TASSEL_0+TACLR+MC_1;     // TA1CLK, Reset, up mode
        TA1CTL &= ~TAIFG;                 // clear IFG        
        while(!(TACTL & TAIFG))
        {
            j++;
        } // end accumulation
        counts[i] = j;   
    }
    // End Sequence
    //** Context Restore
    //  WDTp: IE1, WDCTL
    //  TIMERA0: TACTL, TACCTL1
    //  COMPAp: CACTL1, CACTL2, CAPD
    //  Ports: caoutDIR, caoutSel, txclkDIR, txclkSel, caoutSel2, txclkSel2, refout, refdir  
    #ifdef SEL2REGISTER  
    *(group->caoutSel2Register) = contextSaveCaoutSel2;
    *(group->txclkSel2Register) = contextSaveTxclkSel2;       
    #endif    
    TA1CTL = contextSaveTA1CTL;
    TA1CCTL0 = contextSaveTA1CCTL0;
    TA1CCR0 = contextSaveTA1CCR0;
    CACTL1 = contextSaveCACTL1;
    CACTL2 = contextSaveCACTL2;
    CAPD = contextSaveCAPD;
    *(group->caoutDirRegister) = contextSaveCaoutDir;
    *(group->caoutSelRegister) = contextSaveCaoutSel;  
    *(group->txclkDirRegister) = contextSavetxclkDir;
    *(group->txclkSelRegister) = contextSavetxclkSel;    
    *(group->refPxdirRegister) = contextSaveRefDir;
    *(group->refPxoutRegister) = contextSaveRefOutSel;  
}
#endif
#ifdef RC_PAIR_TA0
/*!
 *  @brief   RC method capactiance measurement using a Pair of GPIO and TimerA0
 *
 *           Schematic Description of two GPIO forming RC measruement.
 *  \n       <- Output
 *  \n       -> Input
 *  \n       R  Resistor (typically 1Mohms)
 *   
 *                       +-<-Px.y (reference)
 *                       |
 *                       R
 *                       |
 *             Element---+-->Pa.b                               
 *          
 *             Charge and Discharge Cycle
 *                          + 
 *                      +    +
 *                   +          +
 *                 +                +
 *                +                     +
 *  \n       Start Timer                  After n cycles Stop Timer
 *           The TAR reister value is the number of SMCLK periods within n 
 *           charge and discharge cycles.  This value is directly proportional  
 *           to the capacitance of the element measured. 'n' is defined by the  
 *           variable accumulation_cycles.
 * 
 *  @param   group Address of the structure describing the Sensor to be measured
 *  @param   counts Address to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_RC_PAIR_TA0_HAL(const struct Sensor *group,uint16_t *counts)
{
    uint8_t i;
	uint16_t j;
    
    //** Context Save
//  TIMERA0: TA0CTL
//  Port: inputPxout, inputPxdir, referencePxout, referencePxdir

    uint8_t contextSaveinputPxout,contextSaveinputPxdir,contextSavereferencePxout;
    uint8_t contextSavereferencePxdir;

    #ifdef __MSP430_HAS_SFR__
    uint16_t contextSaveTA0CTL,contextSaveTA0CCR0;

    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCR0 = TA0CCR0;
    #else
    uint16_t contextSaveTACTL,contextSaveTACCR0;
    
    contextSaveTACTL = TACTL; 
    contextSaveTACCR0 = TACCR0; 
    #endif

//** Setup Measurement timer****************************************************
// Choices are TA0,TA1,TB0,TB1,TD0,TD1 these choices are pushed up into the 
// capacitive touch layer.
    #ifdef __MSP430_HAS_SFR__
    TA0CCR0 = 0xFFFF;                           
    #else
    TACCR0 = 0xFFFF;                           
    #endif
    for (i = 0; i<(group->numElements); i++)
    {
        // Context Save
        contextSaveinputPxout = *((group->arrayPtr[i])->inputPxoutRegister);
        contextSaveinputPxdir = *((group->arrayPtr[i])->inputPxdirRegister);
        contextSavereferencePxout = *((group->arrayPtr[i])->referencePxoutRegister);
        contextSavereferencePxdir = *((group->arrayPtr[i])->referencePxdirRegister);
        j = (group->accumulationCycles);
        #ifdef __MSP430_HAS_SFR__
	    TA0CTL = TASSEL_2+TACLR;                // SMCLK, up mode
        #else
        TACTL = TASSEL_2+TACLR;                // SMCLK, up mode
        #endif
        while(j--)
	    {
        //******************************************************************************
	    // Positive cycle
        //    SENSOR ---+---- Input (low to high)
	    //              R
	    //              +---- Rerefence (high)
            //******************************************************************************
	    // Input low
            *((group->arrayPtr[i])->inputPxoutRegister) &= ~((group->arrayPtr[i])->inputBits);
	    *((group->arrayPtr[i])->inputPxdirRegister) |= (group->arrayPtr[i])->inputBits;
            // Reference High
            *((group->arrayPtr[i])->referencePxdirRegister) |= (group->arrayPtr[i])->referenceBits;
            *((group->arrayPtr[i])->referencePxoutRegister) |= ((group->arrayPtr[i])->referenceBits);
            // Wait until low
	    while((*((group->arrayPtr[i])->inputPxinRegister)) & ((group->arrayPtr[i])->inputBits));
            // Change to an input
            *((group->arrayPtr[i])->inputPxdirRegister) &= ~(group->arrayPtr[i])->inputBits;
	    //**************************************************************************
	    // This mechanism is traditianally an LPM with the ISR calculating the 
	    // delta between when the first snapshot and the ISR event.  If this is
	    // included within the library the entire port ISR would not be available
            // to the calling application.  In this example the polling is done with the
            // CPU at expense of power and MIPS but preserves the port ISR for other 
	    // interruptible functions.
	    //**************************************************************************
            #ifdef __MSP430_HAS_SFR__
            TA0CTL |= MC_1;                     // start timer    
            #else
            TACTL |= MC_1;                     // start timer    
            #endif
	    //wait until voltage reaches Vih of port
	    while(!((*((group->arrayPtr[i])->inputPxinRegister) & (group->arrayPtr[i])->inputBits)));
            #ifdef __MSP430_HAS_SFR__
            TA0CTL &= ~ MC_3;                     // stop timer        
            #else
            TACTL &= ~ MC_3;                     // stop timer        
            #endif
			//******************************************************************************
			// Negative cycle
			//    SENSOR ---+---- Input (high to low)
			//              R
			//              +---- Rerefence (low)
			//******************************************************************************
	    // Input High
            *((group->arrayPtr[i])->inputPxoutRegister) |= ((group->arrayPtr[i])->inputBits);  
            *((group->arrayPtr[i])->inputPxdirRegister) |= (group->arrayPtr[i])->inputBits; 
	    // Reference Low
	    *((group->arrayPtr[i])->referencePxoutRegister) &= ~((group->arrayPtr[i])->referenceBits);
	    // Change to an input
	    *((group->arrayPtr[i])->inputPxdirRegister) &= ~((group->arrayPtr[i])->inputBits);
            #ifdef __MSP430_HAS_SFR__
            TA0CTL |= MC_1;                     // start timer  
            #else
            TACTL |= MC_1;                     // start timer  
            #endif
	    //wait until voltage reaches Vil of port  
	    while((*((group->arrayPtr[i])->inputPxinRegister)) & ((group->arrayPtr[i])->inputBits));
            #ifdef __MSP430_HAS_SFR__
            TA0CTL &= ~ MC_3;                     // stop timer        
            #else
            TACTL &= ~ MC_3;                     // stop timer        
            #endif
        } // END accumulation loop for a single element
        #ifdef __MSP430_HAS_SFR__
        counts[i] = TA0R;    
        #else
        counts[i] = TAR;    
        #endif
        // Context Restore
        *((group->arrayPtr[i])->inputPxoutRegister) = contextSaveinputPxout;
        *((group->arrayPtr[i])->inputPxdirRegister) = contextSaveinputPxdir;     
        *((group->arrayPtr[i])->referencePxoutRegister) = contextSavereferencePxout;
        *((group->arrayPtr[i])->referencePxdirRegister) = contextSavereferencePxdir;             
    } // END FOR loop which cycles through elements within sensor
    
    //** Context Restore
    #ifdef __MSP430_HAS_SFR__
    TA0CTL = contextSaveTA0CTL;
    TA0CCR0 = contextSaveTA0CCR0;
    #else
    TACTL = contextSaveTACTL;
    TACCR0 = contextSaveTACCR0;
    #endif

}
#endif
#ifdef fRO_PINOSC_TA0_SW
/*!
 *  @brief   fRO method capactiance measurement using the PinOsc and TimerA0
 *          
 *           Charge and Discharge Cycle 
 *                           + 
 *                       +    +
 *                   +          +
 *                +                +
 *              +                     +
 *           Start Timer             After n cycles Stop Timer
 *  \n       The TAR reister value is the number of SW loops (function of MCLK) 
 *           within n charge and discharge cycles.  This value is directly 
 *           proportional to the capacitance of the element measured. 'n' is 
 *           defined by the variable accumulation_cycles.
 * 
 *  @param   group Address of the structure describing the Sensor to be measured
 *  @param   counts Address to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_fRO_PINOSC_TA0_SW_HAL(const struct Sensor *group,uint16_t *counts)
{ 
    uint8_t i;
    uint16_t j;
//** Context Save
//  TIMERA0: TA0CTL
//  Ports: PxSEL, PxSEL2 
    uint16_t contextSaveTA0CTL, contextSaveTA0CCTL0;
    uint8_t contextSaveSel,contextSaveSel2;
    
    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCTL0 = TA0CCTL0;
    // Setup Measurement timer
    TACCR0 =(group->accumulationCycles);
    for (i =0; i< (group->numElements); i++)
    {
        j = 0;
        // Context Save
        contextSaveSel = *((group->arrayPtr[i])->inputPxselRegister);
        contextSaveSel2 = *((group->arrayPtr[i])->inputPxsel2Register);
        // start single oscillation (rise then fall and trigger on fall)
        *((group->arrayPtr[i])->inputPxselRegister) &= ~((group->arrayPtr[i])->inputBits);
        *((group->arrayPtr[i])->inputPxsel2Register) |= ((group->arrayPtr[i])->inputBits);  
        TA0CTL = TASSEL_3+TACLR+MC_1;      // INCLK, reset, up mode   
        TA0CTL &= ~TAIFG;                  // clear IFG                
        // start timer in up mode
        while(!(TA0CTL & TAIFG))
        {
            j++;
        } // end accumulation
        counts[i] = j;   
        TA0CTL &= ~MC_1;
        // Context Restore
        *((group->arrayPtr[i])->inputPxselRegister) = contextSaveSel;
        *((group->arrayPtr[i])->inputPxsel2Register) = contextSaveSel2;                        
    }
    // End Sequence
    // Context Restore
    TA0CTL = contextSaveTA0CTL;
    TA0CCTL0 = contextSaveTA0CCTL0;
}
#endif 

#ifdef RO_PINOSC_TA0_WDTp
/*!
 *  @brief   RO method capactiance measurement with PinOsc IO, TimerA0, and WDT+
 *
 *  \n       Schematic Description: 
 * 
 *  \n       element-----+->Px.y
 * 
 *  \n       The WDT+ interval represents the measurement window.  The number of 
 *           counts within the TA0R that have accumulated during the measurement
 *           window represents the capacitance of the element.
 * 
 *  @param   group Pointer to the structure describing the Sensor to be measured
 *  @param   counts Pointer to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_RO_PINOSC_TA0_WDTp_HAL(const struct Sensor *group,uint16_t *counts)
{
    uint8_t i;

//** Context Save
//  Status Register:
//  WDTp: IE1, WDTCTL
//  TIMERA0: TA0CTL, TA0CCTL1
//  Ports: PxSEL, PxSEL2
    uint8_t contextSaveSR;
    uint8_t contextSaveIE1;
    uint16_t contextSaveWDTCTL;
    uint16_t contextSaveTA0CTL,contextSaveTA0CCTL1,contextSaveTA0CCR1;
    uint8_t contextSaveSel,contextSaveSel2;

    contextSaveSR = __get_SR_register();
    contextSaveIE1 = IE1;
    contextSaveWDTCTL = WDTCTL;
    contextSaveWDTCTL &= 0x00FF;
    contextSaveWDTCTL |= WDTPW;
    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCTL1 = TA0CCTL1;
    contextSaveTA0CCR1 = TA0CCR1;

//** Setup Measurement timer***************************************************
// Choices are TA0,TA1,TB0,TB1,TD0,TD1 these choices are pushed up into the
// capacitive touch layer.

 // Configure and Start Timer
    TA0CTL = TASSEL_3+MC_2;                // INCLK, cont mode
    TA0CCTL1 = CM_3+CCIS_2+CAP+SCS;       // Pos&Neg,GND,Cap, Sync
    IE1 |= WDTIE;                         // enable WDT interrupt
    for (i = 0; i<(group->numElements); i++)
    {
        // Context Save
        contextSaveSel = *((group->arrayPtr[i])->inputPxselRegister);
        contextSaveSel2 = *((group->arrayPtr[i])->inputPxsel2Register);
	      // Configure Ports for relaxation oscillator
	      *((group->arrayPtr[i])->inputPxselRegister) &= ~((group->arrayPtr[i])->inputBits);
	      *((group->arrayPtr[i])->inputPxsel2Register) |= ((group->arrayPtr[i])->inputBits);
        //**  Setup Gate Timer ********************************************************
	      // Set duration of sensor measurment
	      //WDTCTL = (WDTPW+WDTTMSEL+group->measGateSource+group->accumulationCycles);
        WDTCTL = (WDTPW+WDTTMSEL+(group->measGateSource)+(group->accumulationCycles));
        TA0CTL |= TACLR;                     // Clear Timer_A TAR
        if(group->measGateSource == GATE_WDT_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE);   // Wait for WDT interrupt
        }
        else
        {
	          __bis_SR_register(LPM0_bits+GIE);   // Wait for WDT interrupt
        }
	      TA0CCTL1 ^= CCIS0;                   // Create SW capture of CCR1
	      counts[i] = TA0CCR1;                 // Save result
	      WDTCTL = WDTPW + WDTHOLD;           // Stop watchdog timer
        // Context Restore
        *((group->arrayPtr[i])->inputPxselRegister) = contextSaveSel;
        *((group->arrayPtr[i])->inputPxsel2Register) = contextSaveSel2;
    }
    // End Sequence
    // Context Restore
    __bis_SR_register(contextSaveSR);
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);   //
    }
    IE1 = contextSaveIE1;
    WDTCTL = contextSaveWDTCTL;
    TA0CTL = contextSaveTA0CTL;
    TA0CCTL1 = contextSaveTA0CCTL1;
    TA0CCR1 = contextSaveTA0CCR1;
}
#endif

#ifdef RO_PINOSC_TA0 
/*!
 *  @brief   RO method capactiance measurement using PinOsc IO, and TimerA0
 *
 *  \n       Schematic Description: 
 * 
 *  \n       element-----+->Px.y
 * 
 *  \n       The measurement window is accumulation_cycles/ACLK. The ACLK is
 *           used to generate a capture event via the internal connection CCIOB. 
 *           The counts within the TA0R that have accumulated during the 
 *           measurement window represents the capacitance of the element.
 * 
 *  @param   group Pointer to the structure describing the Sensor to be measured
 *  @param   counts Pointer to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_RO_PINOSC_TA0_HAL(const struct Sensor *group,uint16_t *counts)
{ 
    uint8_t i;
	uint16_t j;
    //** Context Save
//  TIMERA0: TA0CTL, TA0CCTL0
//  Ports: PxSEL, PxSEL2 
    uint16_t contextSaveTA0CTL,contextSaveTA0CCTL0,contextSaveTA0CCR0;
    uint8_t contextSaveSel,contextSaveSel2;

    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCTL0 = TA0CCTL0;
    contextSaveTA0CCR0 = TA0CCR0;

	//** Setup Measurement timer***************************************************
	// Choices are TA0,TA1,TB0,TB1,TD0,TD1 these choices are pushed up into the 
	// capacitive touch layer.
 
	// Configure and Start Timer
	TA0CTL = TASSEL_3+MC_2;                // TACLK, cont mode
	for (i =0; i< (group->numElements); i++)
	{
        // Context Save
        contextSaveSel = *((group->arrayPtr[i])->inputPxselRegister);
        contextSaveSel2 = *((group->arrayPtr[i])->inputPxsel2Register);
	    // Configure Ports for relaxation oscillator
	    j = (group->accumulationCycles);
	    *((group->arrayPtr[i])->inputPxselRegister) &= ~((group->arrayPtr[i])->inputBits);
	    *((group->arrayPtr[i])->inputPxsel2Register) |= ((group->arrayPtr[i])->inputBits);
	    TA0CCTL0 = CM_3+CCIS_1+CAP;            // Pos&Neg,ACLK (CCI0B),Cap
	    while(!(TA0CCTL0 & CCIFG));             // wait for capture event  
	    TA0CTL |= TACLR;                       // Clear Timer_A TAR
		while(j--)
		{
            TA0CCTL0 = CM_3+CCIS_1+CAP;            // Pos&Neg,ACLK (CCI0B),Cap
            while(!(TA0CCTL0 & CCIFG));             // wait for capture event 
		} 
	    counts[i] = TA0CCR0;                    // Save result
        TA0CTL = TASSEL_3+MC_2;                        
        // Context Restore
        *((group->arrayPtr[i])->inputPxselRegister) = contextSaveSel;
        *((group->arrayPtr[i])->inputPxsel2Register) = contextSaveSel2;
    }
    // End Sequence
    // Context Restore
    TA0CTL = contextSaveTA0CTL;
    TA0CCTL0 = contextSaveTA0CCTL0;
    TA0CCR0 = contextSaveTA0CCR0;
}
#endif

#ifdef RO_COMPB_TA0_WDTA
/*!
 *  @brief   RO method capactiance measurement using CompB, TimerA0, and WDTA
 *
 *  \n       Schematic Description of CompB forming relaxation oscillator and
 *           coupling (connection) between the relaxation oscillator and 
 *           TimerA0.
 *  \n       <- Output
 *  \n       -> Input
 *  \n       R  Resistor (typically 100Kohms)
 * 
 *            element---+-R--<-CBOUT/TA1CLK
 *                      |
 *                      +---->CBx                              
 * 
 *  \n      The WDTA interval represents the measurement window.  The number of 
 *          counts within the TA0R that have accumulated during the measurement
 *          window represents the capacitance of the element.
 * 
 *  @param   group Address of the structure describing the Sensor to be measured
 *  @param   counts Address to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_RO_COMPB_TA0_WDTA_HAL(const struct Sensor *group,uint16_t *counts)
{ 
   uint8_t i;
    
//** Context Save
//  Status Register: 
//  WDTA: IE1, WDTCTL
//  TIMERA0: TA0CTL, TA0CCTL1
//  COMPAp: CACTL1, CACTL2, CAPD
//  Ports: CboutDIR, CboutSel 
    uint8_t contextSaveSR; 
    uint16_t contextSaveSFRIE1;
    uint16_t contextSaveWDTCTL;
    uint16_t contextSaveTA0CTL,contextSaveTA0CCTL1,contextSaveTA0CCR1;
    uint16_t contextSaveCBCTL0,contextSaveCBCTL1;
    uint16_t contextSaveCBCTL2,contextSaveCBCTL3;
    uint8_t contextSaveCboutDir,contextSaveCboutSel;  

    contextSaveSR = __get_SR_register();
    contextSaveSFRIE1 = SFRIE1;
    contextSaveWDTCTL = WDTCTL;
    contextSaveWDTCTL &= 0x00FF;
    contextSaveWDTCTL |= WDTPW;        
    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCTL1 = TA0CCTL1;
    contextSaveTA0CCR1 = TA0CCR1;
    
    contextSaveCBCTL0 = CBCTL0;
    contextSaveCBCTL1 = CBCTL1;
    contextSaveCBCTL2 = CBCTL2;
    contextSaveCBCTL3 = CBCTL3;
    contextSaveCboutDir = *(group->cboutTAxDirRegister);
    contextSaveCboutSel = *(group->cboutTAxSelRegister);  

    //** Setup Measurement timer************************************************
    // connect CBOUT with TA0
    *(group->cboutTAxDirRegister) |= (group->cboutTAxBits);
    *(group->cboutTAxSelRegister) |= (group->cboutTAxBits);

    CBCTL2 = CBREF14+CBREF13 + CBREF02;
  
    // Configure Timer TA0
    TA0CTL = TASSEL_0+MC_2;                // TACLK, cont mode
    TA0CCTL1 = CM_3+CCIS_2+CAP+SCS;            // Pos&Neg,GND,Cap
  
    // Turn on Comparator
    CBCTL1 = CBON;                            // Comparator on without filter
                                              // Vcc to resistor ladder

    CBCTL3 |= (group->cbpdBits);               // set CPD bits to disable 
                                             // I/O buffer
    SFRIE1 |= WDTIE;                           // enable WDT interrupt
    CBCTL2 |= CBRS_1;                          // Turn on reference
    for (i = 0; i<(group->numElements); i++)
    {
        CBCTL0 = CBIMEN + (group->arrayPtr[i])->inputBits;
                                                 
        //**  Setup Gate Timer *************************************************
        // Set duration of sensor measurment
        WDTCTL = WDTPW + WDTTMSEL + group->measGateSource 
		           + group->accumulationCycles;
        TA0CTL |= TACLR;                     // Clear Timer_A TAR
        if(group->measGateSource == GATE_WDTA_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE);   // Wait for WDT interrupt
        }
        else
        {
	          __bis_SR_register(LPM0_bits+GIE);   // Wait for WDT interrupt
        }
	    TA0CCTL1 ^= CCIS0;                   // Create SW capture of CCR1
	    counts[i] = TA0CCR1;                 // Save result
	    WDTCTL = WDTPW + WDTHOLD;           // Stop watchdog timer
    }  
    // End Sequence
    //** Context Restore
    //  WDTA: IE1, WDCTL
    //  TIMERA0: TACTL, TACCTL1
    //  COMPB: CBCTL0, CBCTL1, CBCTL2, CBCTL3
    //  Ports: CboutDIR, CboutSel 
    __bis_SR_register(contextSaveSR);   
    if(!(contextSaveSR & GIE))
    {
      __bic_SR_register(GIE);   // Wait for WDT interrupt        
    }
    SFRIE1 = contextSaveSFRIE1;
    WDTCTL = contextSaveWDTCTL;
    TA0CTL = contextSaveTA0CTL;
    TA0CCTL1 = contextSaveTA0CCTL1;
    TA0CCR1 = contextSaveTA0CCR1;
    CBCTL0 = contextSaveCBCTL0;
    CBCTL1 = contextSaveCBCTL1;
    CBCTL2 = contextSaveCBCTL2;
    CBCTL3 = contextSaveCBCTL3;
    *(group->cboutTAxDirRegister) = contextSaveCboutDir;
    *(group->cboutTAxSelRegister) = contextSaveCboutSel;  
}
#endif

#ifdef RO_COMPB_TA1_WDTA
/*!
 *  @brief   RO method capactiance measurement using CompB, TimerA1, and WDTA
 *
 *  \n      Schematic Description of CompB forming relaxation oscillator and
 *          coupling (connection) between the relaxation oscillator and TimerA1.
 *  \n      <- Output
 *  \n      -> Input
 *  \n      R  Resistor (typically 100Kohms)
 * 
 *          element---+-R--<-CBOUT/TA1CLK
 *                    |
 *                    +---->CBx 
 * 
 *  \n      The WDTA interval represents the measurement window.  The number of 
 *          counts within the TA1R that have accumulated during the measurement
 *          window represents the capacitance of the element.
 * 
 *  @param   group Address of the structure describing the Sensor to be measured
 *  @param   counts Address to where the measurements are to be written
 *  @return  none
 ******************************************************************************/
void TI_CTS_RO_COMPB_TA1_WDTA_HAL(const struct Sensor *group, uint16_t *counts)
{ 
    uint8_t i=0;
    
    //** Context Save
//  Status Register: 
//  WDTA: IE1, WDTCTL
//  TIMERA1: TA1CTL, TA1CCTL1
//  COMPB: CBCTL0, CBCTL1, CBCTL2, CBCTL3
//  Ports: CboutDIR, CboutSel 
    uint8_t contextSaveSR; 
    uint16_t contextSaveSFRIE1;
    uint16_t contextSaveWDTCTL;
    uint16_t contextSaveTA1CTL,contextSaveTA1CCTL1,contextSaveTA1CCR1;
    uint16_t contextSaveCBCTL0,contextSaveCBCTL1;
    uint16_t contextSaveCBCTL2,contextSaveCBCTL3;
    uint8_t contextSaveCboutDir,contextSaveCboutSel;  

    contextSaveSR = __get_SR_register();
    contextSaveSFRIE1 = SFRIE1;
    contextSaveWDTCTL = WDTCTL;
    contextSaveWDTCTL &= 0x00FF;
    contextSaveWDTCTL |= WDTPW;        
    contextSaveTA1CTL = TA1CTL;
    contextSaveTA1CCTL1 = TA1CCTL1;
    contextSaveTA1CCR1 = TA1CCR1;
    
    contextSaveCBCTL0 = CBCTL0;
    contextSaveCBCTL1 = CBCTL1;
    contextSaveCBCTL2 = CBCTL2;
    contextSaveCBCTL3 = CBCTL3;
    contextSaveCboutDir = *(group->cboutTAxDirRegister);
    contextSaveCboutSel = *(group->cboutTAxSelRegister);  

    //** Setup Measurement timer************************************************
    // connect CBOUT with TA1
    *(group->cboutTAxDirRegister) |= (group->cboutTAxBits);
    *(group->cboutTAxSelRegister) |= (group->cboutTAxBits);
    // Setup Comparator
    CBCTL2 = CBREF14+CBREF13 + CBREF02;
    // Configure Timer TA1
    TA1CTL = TASSEL_0+MC_2;                // TACLK, cont mode
    TA1CCTL1 = CM_3+CCIS_2+CAP+SCS;            // Pos&Neg,GND,Cap
    // Turn on Comparator
    CBCTL1 = CBON;                             // Turn on COMPB w/out filter
    CBCTL3 |= (group->cbpdBits);               // set CPD bits to disable 
    SFRIE1 |= WDTIE;                           // enable WDT interrupt
    CBCTL2 |= CBRS_1;                          // Turn on reference
    for (i = 0; i<(group->numElements); i++)
    {
        CBCTL0 = CBIMEN + (group->arrayPtr[i])->inputBits;
        //**  Setup Gate Timer *************************************************
        // Set duration of sensor measurment
	    WDTCTL = WDTPW + WDTTMSEL + WDTCNTCL
	    		+ group->measGateSource + group->accumulationCycles;
	    TA1CTL |= TACLR;                     // Clear Timer_A TAR
        if(group->measGateSource == GATE_WDTA_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE);   // Wait for WDT interrupt
        }
        else
        {
		    __bis_SR_register(LPM0_bits+GIE);   // Wait for WDT interrupt
        }
	    TA1CCTL1 ^= CCIS0;                   // Create SW capture of CCR1
	    counts[i] = TA1CCR1;                 // Save result
	    WDTCTL = WDTPW + WDTHOLD;           // Stop watchdog timer
	    CBCTL3 &= ~((group->arrayPtr[i])->inputBits);
	}
    // End Sequence
    //** Context Restore
    //  WDTA: IE1, WDCTL
    //  TIMERA0: TACTL, TACCTL1
    //  COMPB: CBCTL0, CBCTL1, CBCTL2, CBCTL3
    //  Ports: CboutDIR, CboutSel 
    __bis_SR_register(contextSaveSR);   
    if(!(contextSaveSR & GIE))
    {
      __bic_SR_register(GIE);   // Wait for WDT interrupt        
    }
    SFRIE1 = contextSaveSFRIE1;
    WDTCTL = contextSaveWDTCTL;
    TA1CTL = contextSaveTA1CTL;
    TA1CCTL1 = contextSaveTA1CCTL1;
    TA1CCR1 = contextSaveTA1CCR1;
    CBCTL0 = contextSaveCBCTL0;
    CBCTL1 = contextSaveCBCTL1;
    CBCTL2 = contextSaveCBCTL2;
    CBCTL3 = contextSaveCBCTL3;
    *(group->cboutTAxDirRegister) = contextSaveCboutDir;
    *(group->cboutTAxSelRegister) = contextSaveCboutSel;  
}
#endif

#ifdef fRO_COMPB_TA0_SW
/*!
 *  @brief   fRO method capactiance measurement using CompB, TimerA0 
 *
 *  \n       Schematic Description of CompB forming relaxation oscillator and
 *           coupling (connection) between the relaxation oscillator and 
 *           TimerA0.
 *  \n       <- Output
 *  \n       -> Input
 *  \n       R  Resistor (typically 100Kohms)
 * 
 *          element---+-R--<-CBOUT/TA1CLK
 *                    |
 *                    +---->CBx                               
 * 
 *  \n      The TAR reister value is the number of SW loops (function of MCLK) 
 *          within n charge and discharge cycles.  This value is directly 
 *          proportional to the capacitance of the element measured. 'n' is 
 *          defined by the variable accumulation_cycles.
 * 
 *  @param   group Address of the structure describing the Sensor to be measured
 *  @param   counts Address to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_fRO_COMPB_TA0_SW_HAL(const struct Sensor *group, uint16_t *counts)
{ 
    uint8_t i;
    uint16_t j;
    
//** Context Save
//  TIMERA0: TA0CTL, TA0CCR1
//  COMPB: CBCTL0, CBCTL1, CBCTL2, CBCTL3
//  Ports: CboutDIR, CboutSel 

    uint16_t contextSaveTA0CTL,contextSaveTA0CCR0;
    uint16_t contextSaveCBCTL0,contextSaveCBCTL1;
    uint16_t contextSaveCBCTL2,contextSaveCBCTL3;
    uint8_t contextSaveCboutDir,contextSaveCboutSel;  

    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCR0 = TA0CCR0;
    
    contextSaveCBCTL0 = CBCTL0;
    contextSaveCBCTL1 = CBCTL1;
    contextSaveCBCTL2 = CBCTL2;
    contextSaveCBCTL3 = CBCTL3;
    contextSaveCboutDir = *(group->cboutTAxDirRegister);
    contextSaveCboutSel = *(group->cboutTAxSelRegister);  
    //** Setup Measurement timer************************************************
    // connect CBOUT with TA0
    *(group->cboutTAxDirRegister) |= (group->cboutTAxBits);
    *(group->cboutTAxSelRegister) |= (group->cboutTAxBits);
    CBCTL2 = CBREF14+CBREF13 + CBREF02;
    // Configure Timer TA0
    TA0CCR0 =(group->accumulationCycles);

    // Turn on Comparator
    CBCTL1 = CBON;                             // Turn on COMPB w/out filter
                                               // Vcc to resistor ladder
    CBCTL3 |= (group->cbpdBits);               // set CPD bits to disable 
                                               // I/O buffer
    CBCTL2 |= CBRS_1;                          // Turn on reference
    for (i = 0; i<(group->numElements); i++)
    {
        j=0;
        CBCTL0 = CBIMEN + (group->arrayPtr[i])->inputBits;                                         
        //**  Setup Gate Timer *************************************************
        // Set duration of sensor measurment
        TA0CTL = TASSEL_0+TACLR+MC_1;             // TACLK
        TA0CTL &= ~TAIFG;             // TACLK        
        while(!(TA0CTL & TAIFG))
        {
            j++;
        } // end accumulation
        counts[i] = j;   
    }
    // End Sequence
    //** Context Restore
    //  TIMERA0: TACTL, TACCTL1
    //  COMPB: CBCTL0, CBCTL1, CBCTL2, CBCTL3
    //  Ports: CboutDIR, CboutSel 
    TA0CTL = contextSaveTA0CTL;
    TA0CCR0 = contextSaveTA0CCR0;
    CBCTL0 = contextSaveCBCTL0;
    CBCTL1 = contextSaveCBCTL1;
    CBCTL2 = contextSaveCBCTL2;
    CBCTL3 = contextSaveCBCTL3;
    *(group->cboutTAxDirRegister) = contextSaveCboutDir;
    *(group->cboutTAxSelRegister) = contextSaveCboutSel;  
}
#endif

#ifdef fRO_COMPB_TA1_SW
/*!
 *  @brief   fRO method capactiance measurement using CompB, TimerA1 
 *
 *  \n       Schematic Description of CompB forming relaxation oscillator and
 *           coupling (connection) between the relaxation oscillator and 
 *           TimerA1.
 *  \n       <- Output
 *  \n       -> Input
 *  \n       R  Resistor (typically 100Kohms)
 * 
 *           element---+-R--<-CBOUT/TA1CLK
 *                     |
 *                     +---->CBx                               
 * 
 *  \n       The TAR reister value is the number of SW loops (function of MCLK) 
 *           within n charge and discharge cycles.  This value is directly 
 *           proportional to the capacitance of the element measured. 'n' is 
 *           defined by the variable accumulation_cycles.
 * 
 *  @param   group Address of the structure describing the Sensor to be measured
 *  @param   counts Address to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_fRO_COMPB_TA1_SW_HAL(const struct Sensor *group,uint16_t *counts)
{ 
    uint8_t i;
    uint16_t j;
//** Context Save
//  TIMERA0: TA1CTL, TA1CCTL1
//  COMPB: CBCTL0, CBCTL1, CBCTL2, CBCTL3
//  Ports: CboutDIR, CboutSel 

    uint16_t contextSaveTA1CTL,contextSaveTA1CCR0;
    uint16_t contextSaveCBCTL0,contextSaveCBCTL1;
    uint16_t contextSaveCBCTL2,contextSaveCBCTL3;
    uint8_t contextSaveCboutDir,contextSaveCboutSel;  

    contextSaveTA1CTL = TA1CTL;
    contextSaveTA1CCR0 = TA1CCR0;
    
    contextSaveCBCTL0 = CBCTL0;
    contextSaveCBCTL1 = CBCTL1;
    contextSaveCBCTL2 = CBCTL2;
    contextSaveCBCTL3 = CBCTL3;
    contextSaveCboutDir = *(group->cboutTAxDirRegister);
    contextSaveCboutSel = *(group->cboutTAxSelRegister);  
    //** Setup Measurement timer************************************************
    // connect CBOUT with TA1
    *(group->cboutTAxDirRegister) |= (group->cboutTAxBits);
    *(group->cboutTAxSelRegister) |= (group->cboutTAxBits);
    CBCTL2 = CBREF14+CBREF13 + CBREF02;
    // Configure Timer TA1
    TA1CCR0 =(group->accumulationCycles);

    // Turn on Comparator
    CBCTL1 = CBON;                             // Turn on COMPB w/out filter
                                               // Vcc to resistor ladder
    CBCTL3 |= (group->cbpdBits);               // set CPD bits to disable 
                                               // I/O buffer
    CBCTL2 |= CBRS_1;                          // Turn on reference
    for (i = 0; i<(group->numElements); i++)
    {
        j=0;
        CBCTL0 = CBIMEN + (group->arrayPtr[i])->inputBits;                                         
        //**  Setup Gate Timer *************************************************
        // Set duration of sensor measurment
        TA1CTL = TASSEL_0+TACLR+MC_1;          // TA1CLK, reset, up mode
        TA1CTL &= ~TAIFG;                      // clear ifg
        while(!(TA1CTL & TAIFG))
        {
            j++;
        } // end accumulation
        counts[i] = j;   
        //P1SEL &=~BIT4; 
    }
    // End Sequence
    //** Context Restore
    //  TIMERA0: TACTL, TACCTL1
    //  COMPB: CBCTL0, CBCTL1, CBCTL2, CBCTL3
    //  Ports: CboutDIR, CboutSel 
    TA1CTL = contextSaveTA1CTL;
    TA1CCR0 = contextSaveTA1CCR0;
    CBCTL0 = contextSaveCBCTL0;
    CBCTL1 = contextSaveCBCTL1;
    CBCTL2 = contextSaveCBCTL2;
    CBCTL3 = contextSaveCBCTL3;
    *(group->cboutTAxDirRegister) = contextSaveCboutDir;
    *(group->cboutTAxSelRegister) = contextSaveCboutSel;  
}
#endif

#ifdef RO_COMPB_TA1_TA0
/*!
 *  ======== TI_CTS_RO_COMPB_TA1_TA0_HAL =======
 *  @brief   RO method capacitance measurement using CompB, TimerA1, and TimerA0
 *
 *  \n       Schematic Description of CompB forming relaxation oscillator and
 *  \n       coupling (connection) between the relaxation oscillator and TimerA1.
 *  \n       <- Output
 *  \n       -> Input
 *  \n       R  Resistor (typically 100Kohms)
 * 
 *           element--+--R--<-CBOUT/TA1CLK
 *                    |
 *                    +----->-CBx
 * 
 *  \n       The TimerA0 interval represents the measurement window.  The number
 *           of counts within TA1R that have accumulated during the
 *           measurement window represents the capacitance of the element.
 * 
 *  @param group  pointer to the sensor to be measured
 *  @param counts pointer to where the measurements are to be written
 *  @return        none
 */
void TI_CTS_RO_COMPB_TA1_TA0_HAL(const struct Sensor *group, uint16_t *counts)
{ 
    uint8_t i=0;
    
    /*!
     *  Allocate Context Save Variables
     *  Status Register: GIE bit only
     *  TIMERA0: TA0CTL, TA0CCTL0, TA0CCR0
     *  TIMERA1: TA1CTL, TA1CCTL0, TA1CCR0
     *  COMPB: CBCTL0,CBCTL1,CBCTL2,CBCTL3
     */
    uint8_t contextSaveSR; 
    uint16_t contextSaveTA0CTL,contextSaveTA0CCTL0,contextSaveTA0CCR0;
    uint16_t contextSaveTA1CTL,contextSaveTA1CCTL0,contextSaveTA1CCR0;
    uint16_t contextSaveCBCTL0,contextSaveCBCTL1;
    uint16_t contextSaveCBCTL2,contextSaveCBCTL3;
    uint8_t contextSaveCboutDir,contextSaveCboutSel;  
    /*
	 *  Perform context save of registers used.
	 */
    contextSaveSR = __get_SR_register();
    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCTL0 = TA0CCTL0;
    contextSaveTA0CCR0 = TA0CCR0;
    contextSaveTA1CTL = TA1CTL;
    contextSaveTA1CCTL0 = TA1CCTL0;
    contextSaveTA1CCR0 = TA1CCR0;
    contextSaveCBCTL0 = CBCTL0;
    contextSaveCBCTL1 = CBCTL1;
    contextSaveCBCTL2 = CBCTL2;
    contextSaveCBCTL3 = CBCTL3;
    contextSaveCboutDir = *(group->cboutTAxDirRegister);
    contextSaveCboutSel = *(group->cboutTAxSelRegister);  
    /*
     *  Connect CBOUT with TA1. This also enables the feedback path for the
     *  Relaxation oscillator.
     */
    *(group->cboutTAxDirRegister) |= (group->cboutTAxBits);
    *(group->cboutTAxSelRegister) |= (group->cboutTAxBits);
    /*
     *  The COMPB reference is set to Vcc and the reference resistor taps are
     *  Vcc*(0x18+1)/32 for CBOUT = 1 and Vcc*((0x04+1)/32 for CBOUT = 0.
     *  If Vcc is 3.0V, then the Vih is 2.34V and the Vil is 0.47V. In the
     *  event that CBOUT is connected to DVIO which is not equal to Vcc, then
     *  these voltage levels need to be adjusted.
     */
    CBCTL2 = CBRS_1 + CBREF14 + CBREF13 + CBREF02;
    CBCTL3 |= (group->cbpdBits);         // set CPD bits to disable digital IO
    /*
     *  TimerA1 is the measurement timer and counts the number of relaxation
     *  oscillation cycles of the element which is connected to TACLK.
     *  TimerA1 is in continuous mode.  TA1CCR0 is configured as a capture
     *  register and will be triggered as a SW capture event.
     */
    TA1CTL = TASSEL_0+MC_2;
    TA1CCTL0 = CM_3+CCIS_2+CAP+SCS;
    /*
     *  TimerA0 is the gate (measurement interval) timer.  The number of
     *  oscillations counted, by TimerA1, within the gate interval represents
     *  the measured capacitance.
     */
    TA0CCR0 = group->accumulationCycles;
    TA0CTL = group->measGateSource + group->sourceScale;
    TA0CCTL0 = CCIE;
    CBCTL1 = CBON;                          // Turn on COMPB w/out filter
    for (i = 0; i<(group->numElements); i++)
    {
        /* Turn on specific comparator input. */
        CBCTL0 = CBIMEN + (group->arrayPtr[i])->inputBits;
        TA1CTL |= TACLR;                     // Clear TimerA1, measurement timer
        TA1CTL &= ~TAIFG;                    // Clear overflow flag
        TA0CTL |= (TACLR + MC_1);            // Clear and start TimerA0
        /*
		 *  The measGateSource represents the gate source for timer TIMERA0,
		 *  which can be sourced from TACLK, ACLK, SMCLK, or INCLK.  The
		 *  interrupt handler is defined in TIMER0_A0_VECTOR, which simply
		 *  clears the low power mode bits in the Status Register before
		 *  returning from the ISR.
		 */
        if(group->measGateSource == TIMER_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE); // Enable GIE and wait for ISR
        }
        else
        {
            __bis_SR_register(LPM0_bits+GIE);
        }
        TA1CCTL0 ^= CCIS0;  // Create SW capture of TA1R into TA1CCR0.
        TA0CTL &= ~MC_1;    // Halt Timer
        if(TA1CTL & TAIFG)
        {
        	/*
			 *  If a rollover in the timer has occurred then set counts to
			 *  0.  This will prevent erroneous data from entering the baseline
			 *  tracking algorithm.
			 */
			counts[i] = 0;
        }
        else
        {
            counts[i] = TA1CCR0;  // Save result
        }
    }  // End For Loop
    /*
     *  Context restore GIE within Status Register and registers used.
     */
    __bis_SR_register(contextSaveSR);   
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);
    }
    TA0CTL = contextSaveTA0CTL;
    TA0CCTL0 = contextSaveTA0CCTL0;
    TA0CCR0 = contextSaveTA0CCR0;
    TA1CTL = contextSaveTA1CTL;
    TA1CCTL0 = contextSaveTA1CCTL0;
    TA1CCR0 = contextSaveTA1CCR0;
    CBCTL0 = contextSaveCBCTL0;
    CBCTL1 = contextSaveCBCTL1;
    CBCTL2 = contextSaveCBCTL2;
    CBCTL3 = contextSaveCBCTL3;
    *(group->cboutTAxDirRegister) = contextSaveCboutDir;
    *(group->cboutTAxSelRegister) = contextSaveCboutSel;  
}
#endif

#ifdef fRO_COMPB_TA1_TA0
/*!
 *  ======== TI_CTS_fRO_COMPB_TA1_TA0_HAL() ========
 *  @brief  fRO method capacitance measurement using CompB, TimerA1, and TimerA0
 *
 *  /n      Schematic Description of CompB forming relaxation oscillator and
 *          coupling (connection) between the relaxation oscillator and TimerA1.
 *  /n      <- Output
 *  /n      -> Input
 *  /n      R  Resistor (typically 100Kohms)
 * 
 *           element--+--R--<-CBOUT/TA1CLK
 *                    |
 *                    +----->-CBx
 * 
 *  /n       The TimerA1 interval represents the measurement window.  The number
 *           of counts within TA0R that have accumulated during the
 *           measurement window represents the capacitance of the element.
 * 
 *  @param (group)  pointer to the sensor to be measured
 *  @param (counts) pointer to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_fRO_COMPB_TA1_TA0_HAL(const struct Sensor *group,uint16_t *counts)
{ 
    uint8_t i=0;

    /*!
     *  Allocate Context Save Variables
     *  Status Register: GIE bit only
     *  TIMERA0: TA0CTL, TA0CCTL0, TA0CCR0
     *  TIMERA1: TA1CTL, TA1CCTL0, TA1CCR0
     *  COMPB: CBCTL0,CBCTL1,CBCTL2,CBCTL3
     */
    uint8_t contextSaveSR;
    uint16_t contextSaveTA0CTL,contextSaveTA0CCTL0,contextSaveTA0CCR0;
    uint16_t contextSaveTA1CTL,contextSaveTA1CCTL0,contextSaveTA1CCR0;
    uint16_t contextSaveCBCTL0,contextSaveCBCTL1;
    uint16_t contextSaveCBCTL2,contextSaveCBCTL3;
    uint8_t contextSaveCboutDir,contextSaveCboutSel;  
    /*
	 *  Perform context save of registers used.
	 */
    contextSaveSR = __get_SR_register();
    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCTL0 = TA0CCTL0;
    contextSaveTA0CCR0 = TA0CCR0;
    contextSaveTA1CTL = TA1CTL;
    contextSaveTA1CCTL0 = TA1CCTL0;
    contextSaveTA1CCR0 = TA1CCR0;
    contextSaveCBCTL0 = CBCTL0;
    contextSaveCBCTL1 = CBCTL1;
    contextSaveCBCTL2 = CBCTL2;
    contextSaveCBCTL3 = CBCTL3;
    contextSaveCboutDir = *(group->cboutTAxDirRegister);
    contextSaveCboutSel = *(group->cboutTAxSelRegister);  
    /*
     *  Connect CBOUT with TA1. This also enables the feedback path for the
     *  Relaxation oscillator.
     */
    *(group->cboutTAxDirRegister) |= (group->cboutTAxBits);
    *(group->cboutTAxSelRegister) |= (group->cboutTAxBits);
    /*
     *  The COMPB reference is set to Vcc and the reference resistor taps are
     *  Vcc*(0x18+1)/32 for CBOUT = 1 and Vcc*((0x04+1)/32 for CBOUT = 0.
     *  If Vcc is 3.0V, then the Vih is 2.34V and the Vil is 0.47V. In the
     *  event that CBOUT is connected to DVIO which is not equal to Vcc, then
     *  these voltage levels need to be adjusted.
     */
    CBCTL2 = CBRS_1 + CBREF14 + CBREF13 + CBREF02;
    CBCTL3 |= (group->cbpdBits);         // set CPD bits to disable digital IO
    /*
     *  TimerA0 is the measurement timer and counts the number of clock cycles
     *  ;typically SMCLK, but can also be TACLK, INCLK, or ACLK.
     *  TimerA0 is in continuous mode.  TA0CCR0 is configured as a capture
     *  register and will be triggered as a SW capture event.
     */
    TA0CTL = group->measGateSource + MC_2;
    TA0CCTL0 = CM_3+CCIS_2+CAP+SCS;
    /*
     *  TimerA1 is the gate (measurement interval) timer.  With the fRO method
     *  the gate time varies with the capacitance of the element.  The number of
     *  clock cycles counted, by TimerA0, within the gate interval represents
     *  the measured capacitance.
     */
    TA1CCR0 = group->accumulationCycles;
    TA1CTL = group->sourceScale;
    TA1CCTL0 = CCIE;
    CBCTL1 = CBON;                          // Turn on COMPB w/out filter
    for (i = 0; i<(group->numElements); i++)
    {
        /*
         *  Turn on specific comparator input.
         */
        CBCTL0 = CBIMEN + (group->arrayPtr[i])->inputBits;
        TA0CTL |= TACLR;                     // Clear TimerA1, measurement timer
        TA0CTL &= ~TAIFG;                    // Clear overflow flag
        TA1CTL |= (TACLR + MC_1);            // Clear and start TimerA0
        /*
		 *  The measGateSource represents the measurement source for timer
		 *  TimerA0, which can be sourced from TACLK, ACLK, SMCLK, or INCLK. The
		 *  interrupt handler is defined in TIMER1_A0_VECTOR, which simply
		 *  clears the low power mode bits in the Status Register before
		 *  returning from the ISR.
		 */
        if(group->measGateSource == TIMER_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE); // Enable GIE and wait for ISR
        }
        else
        {
            __bis_SR_register(LPM0_bits+GIE);
        }
        TA0CCTL0 ^= CCIS0;  // Create SW capture of TA1R into TA1CCR0.
        TA1CTL &= ~MC_1;    // Halt Timer
        if(TA0CTL & TAIFG)
        {
        	/*
			 *  If a rollover in the timer has occurred then set counts to
			 *  0.  This will prevent erroneous data from entering the baseline
			 *  tracking algorithm.
			 */
			counts[i] = 0;
        }
        else
        {
            counts[i] = TA0CCR0;  // Save result
        }
    }  // End For Loop
    /*
     *  Context restore GIE within Status Register and registers used.
     */
    __bis_SR_register(contextSaveSR);
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);
    }
    TA0CTL = contextSaveTA0CTL;
    TA0CCTL0 = contextSaveTA0CCTL0;
    TA0CCR0 = contextSaveTA0CCR0;
    TA1CTL = contextSaveTA1CTL;
    TA1CCTL0 = contextSaveTA1CCTL0;
    TA1CCR0 = contextSaveTA1CCR0;
    CBCTL0 = contextSaveCBCTL0;
    CBCTL1 = contextSaveCBCTL1;
    CBCTL2 = contextSaveCBCTL2;
    CBCTL3 = contextSaveCBCTL3;
    *(group->cboutTAxDirRegister) = contextSaveCboutDir;
    *(group->cboutTAxSelRegister) = contextSaveCboutSel;
}
#endif


#ifdef RO_PINOSC_TA1_WDTp
/*!
 *  @brief   RO method capactiance measurement with PinOsc IO, TimerA1, and WDT+
 *
 *  \n       Schematic Description:
 *
 *  \n       element-----+->Px.y
 *
 *  \n       The WDT+ interval represents the measurement window.  The number of
 *           counts within the TA0R that have accumulated during the measurement
 *           window represents the capacitance of the element.
 *
 *  @param   group Pointer to the structure describing the Sensor to be measured
 *  @param   counts Pointer to where the measurements are to be written
 *  @return  none
 */

void TI_CTS_RO_PINOSC_TA1_WDTp_HAL(const struct Sensor *group,uint16_t *counts)
{
    uint8_t i;

//** Context Save
//  Status Register:
//  WDTp: IE1, WDTCTL
//  TIMERA1: TA1CTL, TA1CCTL1
//  Ports: PxSEL, PxSEL2
    uint8_t contextSaveSR;
    uint8_t contextSaveIE1;
    uint16_t contextSaveWDTCTL;
    uint16_t contextSaveTA1CTL,contextSaveTA1CCTL1,contextSaveTA1CCR1;
    uint8_t contextSaveSel,contextSaveSel2;

    contextSaveSR = __get_SR_register();
    contextSaveIE1 = IE1;
    contextSaveWDTCTL = WDTCTL;
    contextSaveWDTCTL &= 0x00FF;
    contextSaveWDTCTL |= WDTPW;
    contextSaveTA1CTL = TA1CTL;
    contextSaveTA1CCTL1 = TA1CCTL1;
    contextSaveTA1CCR1 = TA1CCR1;

//** Setup Measurement timer***************************************************
// Choices are TA0,TA1,TB0,TB1,TD0,TD1 these choices are pushed up into the
// capacitive touch layer.

 // Configure and Start Timer
    TA1CTL = TASSEL_3+MC_2;                // INCLK, cont mode
    TA1CCTL1 = CM_3+CCIS_2+CAP+SCS;            // Pos&Neg,GND,Cap
    IE1 |= WDTIE;                         // enable WDT interrupt
    for (i = 0; i<(group->numElements); i++)
    {
        // Context Save
        contextSaveSel = *((group->arrayPtr[i])->inputPxselRegister);
        contextSaveSel2 = *((group->arrayPtr[i])->inputPxsel2Register);
	      // Configure Ports for relaxation oscillator
	      *((group->arrayPtr[i])->inputPxselRegister) &= ~((group->arrayPtr[i])->inputBits);
	      *((group->arrayPtr[i])->inputPxsel2Register) |= ((group->arrayPtr[i])->inputBits);
        //**  Setup Gate Timer ********************************************************
	      // Set duration of sensor measurment
	      //WDTCTL = (WDTPW+WDTTMSEL+group->measGateSource+group->accumulationCycles);
        WDTCTL = (WDTPW+WDTTMSEL+(group->measGateSource)+(group->accumulationCycles));
        TA1CTL |= TACLR;                     // Clear Timer_A TAR
        if(group->measGateSource == GATE_WDT_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE);   // Wait for WDT interrupt
        }
        else
        {
	          __bis_SR_register(LPM0_bits+GIE);   // Wait for WDT interrupt
        }
	      TA1CCTL1 ^= CCIS0;                   // Create SW capture of CCR1
	      counts[i] = TA1CCR1;                 // Save result
	      WDTCTL = WDTPW + WDTHOLD;           // Stop watchdog timer
        // Context Restore
        *((group->arrayPtr[i])->inputPxselRegister) = contextSaveSel;
        *((group->arrayPtr[i])->inputPxsel2Register) = contextSaveSel2;
    }
    // End Sequence
    // Context Restore
    __bis_SR_register(contextSaveSR);
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);   //
    }
    IE1 = contextSaveIE1;
    WDTCTL = contextSaveWDTCTL;
    TA1CTL = contextSaveTA1CTL;
    TA1CCTL1 = contextSaveTA1CCTL1;
    TA1CCR1 = contextSaveTA1CCR1;
}
#endif

#ifdef RO_PINOSC_TA1_TB0
/*!
 *  ======== TI_CTS_RO_PINOSC_TA1_TB0_HAL ========
 *  @brief   RO method capacitance measurement using PinOsc IO, TimerA1, and
 *           TimerB0
 *
 *  \n       Schematic Description:
 *
 *  \n       element-----+->Px.y
 *
 *  \n       The TimerA1 interval represents the gate (measurement) time.  The
 *           number of oscillations that have accumulated in TA1R during the
 *           measurement time represents the capacitance of the element.
 *
 *  @param group  pointer to the sensor to be measured
 *  @param counts pointer to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_RO_PINOSC_TA1_TB0_HAL(const struct Sensor *group,uint16_t *counts)
{
    uint8_t i;
    /*!
     *  Allocate Context Save Variables
     *  Status Register: GIE bit only
     *  TIMERA0: TA1CTL, TA1CCTL0, TA1CCR0
     *  TIMERB1: TB0CTL, TB0CCTL0, TB0CCR0
     *  Ports: PxSEL, PxSEL2
     */
    uint8_t contextSaveSR;
    uint16_t contextSaveTA1CTL,contextSaveTA1CCTL0,contextSaveTA1CCR0;
    uint16_t contextSaveTB0CTL,contextSaveTB0CCTL0,contextSaveTB0CCR0;
    uint8_t contextSaveSel,contextSaveSel2;
    /*
     *  Perform context save of registers used except port registers which are
     *  saved and restored within the for loop as each element within the
     *  sensor is measured.
     */
    contextSaveSR = __get_SR_register();
    contextSaveTA1CTL = TA1CTL;
    contextSaveTA1CCTL0 = TA1CCTL0;
    contextSaveTA1CCR0 = TA1CCR0;
    contextSaveTB0CTL = TB0CTL;
    contextSaveTB0CCTL0 = TB0CCTL0;
    contextSaveTB0CCR0 = TB0CCR0;
    /*
     *  TimerA0 is the measurement timer and counts the number of relaxation
     *  oscillation cycles of the electrode which is routed to INCLK.  TA1 is
     *  in continuous mode and sourced from INCLK.
     */
    TA1CTL = TASSEL_3+MC_2;
    TA1CCTL0 = CM_3+CCIS_2+CAP+SCS;            // Setup for SW capture
    /*
     *  TimerA1 is the gate (measurement interval) timer.  The number of
     *  oscillations counted within the gate interval represents the measured
     *  capacitance.
     */
    TB0CCR0 = (group->accumulationCycles);
    // Establish source and scale of timerA1, but halt the timer.
    TB0CTL = group->measGateSource + group->sourceScale;
    TB0CCTL0 = CCIE;  // Enable Interrupt when timer counts to TB0CCR0.
    for (i = 0; i<(group->numElements); i++)
    {
        // Context Save Port Registers
        contextSaveSel = *((group->arrayPtr[i])->inputPxselRegister);
        contextSaveSel2 = *((group->arrayPtr[i])->inputPxsel2Register);
        // Configure Ports for relaxation oscillator
        *((group->arrayPtr[i])->inputPxselRegister)
        		&= ~((group->arrayPtr[i])->inputBits);
        *((group->arrayPtr[i])->inputPxsel2Register)
        		|= ((group->arrayPtr[i])->inputBits);
        TA1CTL |= TACLR;
        TA1CTL &= ~TAIFG;
        TB0CTL |= (TACLR + MC_1);
        /*!
		 *  The measGateSource represents the gate source for timer TIMERA1,
		 *  which can be sourced from TACLK, ACLK, SMCLK, or INCLK.  The
		 *  interrupt handler is defined in TIMER1_A0_VECTOR, which simply
		 *  clears the low power mode bits in the Status Register before
		 *  returning from the ISR.
		 */
        if(group->measGateSource == TIMER_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE);  // Enable GIE and wait for ISR
        }
        else
        {
            __bis_SR_register(LPM0_bits+GIE);  // Enable GIE and wait for ISR
        }
        TA1CCTL0 ^= CCIS0;  // Create SW capture of TA1CCR into TA1CCR0.
        TB0CTL &= ~MC_1;    // Halt Timer
        if(TA1CTL & TAIFG)
        {
        	/*
			 *  If a rollover in the timer has occurred then set counts to
			 *  0.  This will prevent erroneous data from entering the baseline
			 *  tracking algorithm.
			 */
			counts[i] = 0;
        }
        else
        {
            counts[i] = TA1CCR0;  // Save result
        }
        // Context Restore Port Registers
        *((group->arrayPtr[i])->inputPxselRegister) = contextSaveSel;
        *((group->arrayPtr[i])->inputPxsel2Register) = contextSaveSel2;
    } // End for Loop
    /*
     *  Context restore GIE within Status Register and all timer registers
     *  used.
     */
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);
    }
    TA1CTL = contextSaveTA1CTL;
    TA1CCTL0 = contextSaveTA1CCTL0;
    TA1CCR0 = contextSaveTA1CCR0;
    TB0CTL = contextSaveTB0CTL;
    TB0CCTL0 = contextSaveTB0CCTL0;
    TB0CCR0 = contextSaveTB0CCR0;
}
#endif


#ifdef fRO_PINOSC_TA1_TA0
/*!
 *  @brief   fRO method capacitance measurement using PinOsc IO, TimerA1, and
 *          TimerA0
 *
 *  \n       Schematic Description:
 *
 *  \n       element-----+->Px.y
 *
 *
 *  @param   group Pointer to the structure describing the Sensor to be measured
 *  @param   counts Pointer to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_fRO_PINOSC_TA1_TA0_HAL(const struct Sensor *group,uint16_t *counts)
{
    uint8_t i;

    /*
     *  Context Save
     *  Status Register: GIE
     *  TIMERA1: TA1CTL, TA1CCTL0, TA1CCR0
     *  TIMERA0: TA0CTL, TA0CCTL0, TA0CCR0
     *  Ports: PxSEL, PxSEL2
     */
    uint8_t contextSaveSR;
    uint16_t contextSaveTA1CTL,contextSaveTA1CCTL0,contextSaveTA1CCR0;
    uint16_t contextSaveTA0CTL,contextSaveTA0CCTL0,contextSaveTA0CCR0;
    uint8_t contextSaveSel,contextSaveSel2;

    contextSaveSR = __get_SR_register();
    contextSaveTA1CTL = TA1CTL;
    contextSaveTA1CCTL0 = TA1CCTL0;
    contextSaveTA1CCR0 = TA1CCR0;
    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCTL0 = TA0CCTL0;
    contextSaveTA0CCR0 = TA0CCR0;

//** Setup Measurement timer***************************************************
// Choices are TA0,TA1,TB0,TB1,TD0,TD1 these choices are pushed up into the
// capacitive touch layer.

    // Configure Measurement interval with TimerA1
    TA1CCR0 = (group->accumulationCycles);
    /*
     *  INCLK, IDx settings from sourceScale definition
     */
    TA1CTL = TASSEL_3 + group->sourceScale;
    TA1CCTL0 = CCIE;

    // Configure and start measurment timerA0
    TA0CTL = group->measGateSource + MC_2 + TACLR;  // cont
    TA0CCTL0 = CM_3+CCIS_2+CAP+SCS;            // Pos&Neg,GND,Cap

    for (i = 0; i<(group->numElements); i++)
    {
        // Context Save
        contextSaveSel = *((group->arrayPtr[i])->inputPxselRegister);
        contextSaveSel2 = *((group->arrayPtr[i])->inputPxsel2Register);
        // Configure Ports for relaxation oscillator
        *((group->arrayPtr[i])->inputPxselRegister) &= ~((group->arrayPtr[i])->inputBits);
        *((group->arrayPtr[i])->inputPxsel2Register) |= ((group->arrayPtr[i])->inputBits);

        TA0CTL |= TACLR;
    	TA0CTL &= ~TAIFG;
        TA1CTL |= (TACLR + MC_1);  // Clear Timer, Up mode
        /*
         *  In this configuration measGateSource represents the measurement
         *  source for timer TIMERA1, which can be sourced from TACLK, ACLK,
         *  SMCLK, or INCLK.
         */
        if(group->measGateSource == TIMER_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE);
        }
        else
        {
            __bis_SR_register(LPM0_bits+GIE);
        }
        TA0CCTL0 ^= CCIS0;        // Create SW capture of CCR1
        TA1CTL &= ~MC_1;          // Halt Timer
        if(TA0CTL & TAIFG)
        {
            /*
             *  If a rollover in the timer has occurred then set counts to
             *  0.  This will prevent erroneous data from entering the baseline
             *  tracking algorithm.
             */
        	counts[i] = 0;
        }
        else
        {
            counts[i] = TA0CCR0;      // Save result
        }
        // Context Restore
        *((group->arrayPtr[i])->inputPxselRegister) = contextSaveSel;
        *((group->arrayPtr[i])->inputPxsel2Register) = contextSaveSel2;
    } // End for loop
    // Context Restore
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);
    }
    TA1CTL = contextSaveTA1CTL;
    TA1CCTL0 = contextSaveTA1CCTL0;
    TA1CCR0 = contextSaveTA1CCR0;
    TA0CTL = contextSaveTA0CTL;
    TA0CCTL0 = contextSaveTA0CCTL0;
    TA0CCR0 = contextSaveTA0CCR0;
}
#endif

#ifdef fRO_PINOSC_TA1_TB0
/*!
 *  @brief   fRO method capacitance measurement using PinOsc IO, TimerA1, and
 *          TimerB0
 *
 *  \n       Schematic Description:
 *
 *  \n       element-----+->Px.y
 *
 *
 *  @param   group Pointer to the structure describing the Sensor to be measured
 *  @param   counts Pointer to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_fRO_PINOSC_TA1_TB0_HAL(const struct Sensor *group,uint16_t *counts)
{
    uint8_t i;

    /*
     *  Context Save
     *  Status Register: GIE
     *  TIMERA1: TA1CTL, TA1CCTL0, TA1CCR0
     *  TIMERB0: TB0CTL, TB0CCTL0, TB0CCR0
     *  Ports: PxSEL, PxSEL2
     */
    uint8_t contextSaveSR;
    uint16_t contextSaveTA1CTL,contextSaveTA1CCTL0,contextSaveTA1CCR0;
    uint16_t contextSaveTB0CTL,contextSaveTB0CCTL0,contextSaveTB0CCR0;
    uint8_t contextSaveSel,contextSaveSel2;

    contextSaveSR = __get_SR_register();
    contextSaveTA1CTL = TA1CTL;
    contextSaveTA1CCTL0 = TA1CCTL0;
    contextSaveTA1CCR0 = TA1CCR0;
    contextSaveTB0CTL = TB0CTL;
    contextSaveTB0CCTL0 = TB0CCTL0;
    contextSaveTB0CCR0 = TB0CCR0;

//** Setup Measurement timer***************************************************
// Choices are TA0,TA1,TB0,TB1,TD0,TD1 these choices are pushed up into the
// capacitive touch layer.

    // Configure Measurement interval with TimerA1
    TA1CCR0 = (group->accumulationCycles);
    /*
     *  INCLK, IDx settings from sourceScale definition
     */
    TA1CTL = TASSEL_3 + group->sourceScale;
    TA1CCTL0 = CCIE;

    // Configure and start measurment timerA0
    TB0CTL = group->measGateSource + MC_2 + TBCLR;  // cont
    TB0CCTL0 = CM_3+CCIS_2+CAP+SCS;            // Pos&Neg,GND,Cap

    for (i = 0; i<(group->numElements); i++)
    {
        // Context Save
        contextSaveSel = *((group->arrayPtr[i])->inputPxselRegister);
        contextSaveSel2 = *((group->arrayPtr[i])->inputPxsel2Register);
        // Configure Ports for relaxation oscillator
        *((group->arrayPtr[i])->inputPxselRegister) &= ~((group->arrayPtr[i])->inputBits);
        *((group->arrayPtr[i])->inputPxsel2Register) |= ((group->arrayPtr[i])->inputBits);

        TB0CTL |= TBCLR;
    	TB0CTL &= ~TBIFG;
        TA1CTL |= (TACLR + MC_1);  // Clear Timer, Up mode
        /*
         *  In this configuration measGateSource represents the measurement
         *  source for timer TIMERA1, which can be sourced from TACLK, ACLK,
         *  SMCLK, or INCLK.
         */
        if(group->measGateSource == TIMER_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE);
        }
        else
        {
            __bis_SR_register(LPM0_bits+GIE);
        }
        TB0CCTL0 ^= CCIS0;        // Create SW capture of CCR1
        TA1CTL &= ~MC_1;          // Halt Timer
        if(TB0CTL & TBIFG)
        {
            /*
             *  If a rollover in the timer has occurred then set counts to
             *  0.  This will prevent erroneous data from entering the baseline
             *  tracking algorithm.
             */
        	counts[i] = 0;
        }
        else
        {
            counts[i] = TB0CCR0;      // Save result
        }
        // Context Restore
        *((group->arrayPtr[i])->inputPxselRegister) = contextSaveSel;
        *((group->arrayPtr[i])->inputPxsel2Register) = contextSaveSel2;
    } // End for loop
    // Context Restore
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);
    }
    TA1CTL = contextSaveTA1CTL;
    TA1CCTL0 = contextSaveTA1CCTL0;
    TA1CCR0 = contextSaveTA1CCR0;
    TB0CTL = contextSaveTB0CTL;
    TB0CCTL0 = contextSaveTB0CCTL0;
    TB0CCR0 = contextSaveTB0CCR0;
}
#endif

#ifdef RO_CSIO_TA0_TA1 
/*!
 *  ======== TI_CTS_RO_CSIO_TA0_TA1_HAL ========
 *  @brief   RO method using Capacitive Touch IO, TimerA0, and TimerA1
 *
 *  \n      Schematic Description: 
 *  \n      element-----+->Px.y
 * 
 *  \n      The TimerA1 interval represents the measurement window.  The number 
 *          of counts within the TA0R that have accumulated during the 
 *          measurement window represents the capacitance of the element.
 * 
 *  @param group  pointer to the sensor to be measured
 *  @param counts pointer to where the measurements are to be written
 *  @return        none
 */
void TI_CTS_RO_CSIO_TA0_TA1_HAL(const struct Sensor *group,uint16_t *counts)
{ 
    uint8_t i;
    /*
     *  Allocate Context Save Variables
     *  Status Register: GIE bit only
     *  TIMERA0: TA0CTL, TA0CCTL0, TA0CCR0
     *  TIMERA1: TA1CTL, TA1CCTL0, TA1CCR0
     *  CSIO: CSIOxCTL
     */
    uint8_t contextSaveSR; 
    uint16_t contextSaveTA0CTL,contextSaveTA0CCTL0,contextSaveTA0CCR0;
    uint16_t contextSaveTA1CTL,contextSaveTA1CCTL0,contextSaveTA1CCR0;
    uint8_t contextSaveCtl;
    /* Perform context save of registers used. */
    contextSaveSR = __get_SR_register();
    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCTL0 = TA0CCTL0;
    contextSaveTA0CCR0 = TA0CCR0;
    contextSaveTA1CTL = TA1CTL;
    contextSaveTA1CCTL0 = TA1CCTL0;
    contextSaveTA1CCR0 = TA1CCR0;
	contextSaveCtl = *(group->inputCapsioctlRegister);
    /*
     *  TimerA0 is the measurement timer and counts the number of relaxation
     *  oscillation cycles of the element which is connected to TA0CLK.
     *  TimerA0 is in continuous mode.  TA0CCR0 is configured as a capture
     *  register and will be triggered as a SW capture event.
     */
    TA0CTL = TASSEL_3+MC_2;
    TA0CCTL0 = CM_3+CCIS_2+CAP+SCS;
    /*
     *  TimerA1 is the gate (measurement interval) timer.  The number of
     *  oscillations counted, by TimerA0, within the gate interval represents
     *  the measured capacitance.
     */
    TA1CCR0 = (group->accumulationCycles);
    TA1CTL = group->measGateSource + group->sourceScale;
    TA1CCTL0 = CCIE;
    for (i = 0; i<(group->numElements); i++)
    {
    	/* Enable Capacitive Touch IO oscillation */
	    *(group->inputCapsioctlRegister)
	            = ((group->arrayPtr[i])->inputBits)+CAPSIOEN;
        TA0CTL |= TACLR;                   // Clear TimerA0, measurement timer
        TA0CTL &= ~TAIFG;                  // Clear overflow flag
        TA1CTL |= (TACLR + MC_1);          // Clear and start TimerA1
        /*
		 *  The measGateSource represents the gate source for timer TIMERA1,
		 *  which can be sourced from TACLK, ACLK, SMCLK, or INCLK.  The
		 *  interrupt handler is defined in TIMER1_A0_VECTOR, which simply
		 *  clears the low power mode bits in the Status Register before
		 *  returning from the ISR.
		 */
        if(group->measGateSource == TIMER_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE);  // Enable GIE and wait for ISR
        }
        else
        {
            __bis_SR_register(LPM0_bits+GIE);
        }
        TA0CCTL0 ^= CCIS0;  // Create SW capture of TA0R into TA0CCR0
        TA1CTL &= ~MC_1;    // Halt Timer
        if(TA0CTL & TAIFG)
        {
        	/*
			 *  If a rollover in the timer has occurred then set counts to
			 *  0.  This will prevent erroneous data from entering the baseline
			 *  tracking algorithm.
			 */
			counts[i] = 0;
        }
        else
        {
            counts[i] = TA0CCR0;  // Save result
        }
    } // End For Loop
    /* Context restore GIE within Status Register and registers used. */
    *(group->inputCapsioctlRegister) = contextSaveCtl;
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);
    }
    TA0CTL = contextSaveTA0CTL;
    TA0CCTL0 = contextSaveTA0CCTL0;
    TA0CCR0 = contextSaveTA0CCR0;
    TA1CTL = contextSaveTA1CTL;
    TA1CCTL0 = contextSaveTA1CCTL0;
    TA1CCR0 = contextSaveTA1CCR0;
}
#endif


#ifdef fRO_CSIO_TA0_TA1
/*!
 *  ======== TI_CTS_fRO_CSIO_TA0_TA1_HAL ========
 *  @brief   fRO method using Capacitive Touch IO, TimerA0, and TimerA1
 *
 *  \n      Schematic Description: 
 *  \n      element-----+->Px.y
 * 
 *  \n      The number of counts in TA0R is configured with the accumulationCycles. 
 *          The number of counts in TA1CCR0 represents the measuring result
 * 
 *  @param group  pointer to the sensor to be measured
 *  @param counts pointer to where the measurements are to be written
 *  @return        none
 */
void TI_CTS_fRO_CSIO_TA0_TA1_HAL(const struct Sensor *group,uint16_t *counts)
{ 
    uint8_t i;
    /*
     *  Allocate Context Save Variables
     *  Status Register: GIE bit only
     *  TIMERA0: TA0CTL, TA0CCTL0, TA0CCR0
     *  TIMERA1: TA1CTL, TA1CCTL0, TA1CCR0
     *  CSIO: CSIOxCTL
     */
    uint8_t contextSaveSR; 
    uint16_t contextSaveTA0CTL,contextSaveTA0CCTL0,contextSaveTA0CCR0;
    uint16_t contextSaveTA1CTL,contextSaveTA1CCTL0,contextSaveTA1CCR0;
    uint8_t contextSaveCtl;
    /* Perform context save of registers used. */
    contextSaveSR = __get_SR_register();
    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCTL0 = TA0CCTL0;
    contextSaveTA0CCR0 = TA0CCR0;
    contextSaveTA1CTL = TA1CTL;
    contextSaveTA1CCTL0 = TA1CCTL0;
    contextSaveTA1CCR0 = TA1CCR0;
    contextSaveCtl = *(group->inputCapsioctlRegister);
    /*
     *  TimerA1 is the measurement timer and counts the number of clock cycles
     *  of the source which is connected to measGateSource, typically SMCLK.
     *  TimerA1 is in continuous mode.  TA1CCR0 is configured as a capture
     *  register and will be triggered as a SW capture event.
     */
    TA1CTL = group->measGateSource + MC_2;
    TA1CCTL0 = CM_3+CCIS_2+CAP+SCS;
    /*
     *  TimerA0 is the gate (measurement interval) timer.  The number of
     *  oscillations counted, by TimerA1, within the gate interval represents
     *  the measured capacitance.  The input is TA0CLK.
     */
    TA0CCR0 = (group->accumulationCycles);
    TA0CTL = TASSEL_3+group->sourceScale;
    TA0CCTL0 = CCIE;
    for (i = 0; i<(group->numElements); i++)
    {
        /* Enable Capacitive Touch IO oscillation */
        *(group->inputCapsioctlRegister)
	            = ((group->arrayPtr[i])->inputBits)+CAPSIOEN;
        TA1CTL |= TACLR;                   // Clear TimerA1, measurement timer
        TA1CTL &= ~TAIFG;                  // Clear overflow flag
        TA0CTL |= (TACLR + MC_1);          // Clear and start TimerA0
        /*
         *  The measGateSource represents the measurement source for timer
         *  TIMERA1, which can be sourced from TACLK, ACLK, SMCLK, or INCLK.
         *  The interrupt handler is defined in TIMER0_A0_VECTOR, which simply
         *  clears the low power mode bits in the Status Register before
         *  returning from the ISR.
         */
        if(group->measGateSource == TIMER_ACLK)
        {
            __bis_SR_register(LPM3_bits+GIE);  // Enable GIE and wait for ISR
        }
        else
        {
            __bis_SR_register(LPM0_bits+GIE);
        }
        TA1CCTL0 ^= CCIS0;  // Create SW capture of TA1R into TA1CCR0
        TA0CTL &= ~MC_1;    // Halt Timer
        if(TA1CTL & TAIFG)
        {
            /*
             *  If a rollover in the timer has occurred then set counts to
             *  0.  This will prevent erroneous data from entering the baseline
             *  tracking algorithm.
             */
             counts[i] = 0;
        }
        else
        {
            counts[i] = TA1CCR0;  // Save result
        }
    } // End For Loop
    /* Context restore GIE within Status Register and registers used. */
    *(group->inputCapsioctlRegister) = contextSaveCtl;
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);
    }
    TA0CTL = contextSaveTA0CTL;
    TA0CCTL0 = contextSaveTA0CCTL0;
    TA0CCR0 = contextSaveTA0CCR0;
    TA1CTL = contextSaveTA1CTL;
    TA1CCTL0 = contextSaveTA1CCTL0;
    TA1CCR0 = contextSaveTA1CCR0;
}
#endif

#ifdef fRO_CSIO_TA0_SW
/*!
 *  @brief   fRO method capactiance measurement using the CSIO and TimerA0
 *          
 *           Charge and Discharge Cycle 
 *                           + 
 *                       +    +
 *                   +          +
 *                +                +
 *              +                     +
 *           Start Timer             After n cycles Stop Timer
 *  \n       The TAR reister value is the number of SW loops (function of MCLK) 
 *           within n charge and discharge cycles.  This value is directly 
 *           proportional to the capacitance of the element measured. 'n' is 
 *           defined by the variable accumulation_cycles.
 * 
 *  @param   group Address of the structure describing the Sensor to be measured
 *  @param   counts Address to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_fRO_CSIO_TA0_SW_HAL(const struct Sensor *group,uint16_t *counts)
{ 
    uint8_t i;
    uint16_t j;
//** Context Save
//  TIMERA0: TA0CTL, TA0CCTL0
//  CSIO Control Register: CSIOxCtl
    uint16_t contextSaveTA0CTL, contextSaveTA0CCTL0;
    uint8_t contextSaveCtl;
    
    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCTL0 = TA0CCTL0;
    // Setup Measurement timer
    TA0CCR0 =(group->accumulationCycles);
	contextSaveCtl = *(group->inputCapsioctlRegister);
    for (i =0; i< (group->numElements); i++)
    {
        j = 0;
        // Context Save
        // start single oscillation (rise then fall and trigger on fall)
        *(group->inputCapsioctlRegister) = 	((group->arrayPtr[i])->inputBits)+CAPSIOEN;	
        TA0CTL = TASSEL_3+TACLR+MC_1;      // INCLK, reset, up mode   
        TA0CTL &= ~TAIFG;                  // clear IFG                
        // start timer in up mode
        while(!(TA0CTL & TAIFG))
        {
            j++;
        } // end accumulation
        counts[i] = j;
        TA0CTL &= ~MC_1;
    }
    // End Sequence
    // Context Restore
    *(group->inputCapsioctlRegister) = contextSaveCtl;
    TA0CTL = contextSaveTA0CTL;
    TA0CCTL0 = contextSaveTA0CCTL0;
}
#endif


#ifdef RO_CSIO_TA0_RTC
/*!
 *  ======== TI_CTS_RO_CSIO_TA0_RTC_HAL ========
 *  @brief  RO method measurement using Capacitive Touch IO, TimerA0, and RTC
 *
 *  \n      Schematic Description:
 *
 *  \n      element-----+->Px.y
 *
 *  \n      The RTC interval represents the measurement window.  The number of
 *          counts within the TA0R that have accumulated during the measurement
 *          window represents the capacitance of the element. RTC connection to
 *          the timer is a Hardware connection defined in MSP430FR4133 
 *          datasheet.
 *
 *  @param group  pointer to the sensor to be measured
 *  @param counts pointer to where the measurements are to be written
 *  @return  none
 */
void TI_CTS_RO_CSIO_TA0_RTC_HAL(const struct Sensor *group,uint16_t *counts)
{
    uint8_t i;
    /*
     *  Allocate Context Save Variables
     *  Status Register: GIE bit only
     *  RTC: RTCCTL
     *  TIMERA0: TA0CTL, TA0CCTL0, TA0CCR0
     *  CSIO: CSIOxCTL
     */
    uint8_t contextSaveSR;
    uint16_t contextSaveRTCCTL;
    uint16_t contextSaveTA0CTL,contextSaveTA0CCTL0,contextSaveTA0CCR0;
    uint8_t contextSaveCtl;
    /* Perform context save of registers used. */
    contextSaveSR = __get_SR_register();
    contextSaveRTCCTL = RTCCTL;
    contextSaveTA0CTL = TA0CTL;
    contextSaveTA0CCTL0 = TA0CCTL0;
    contextSaveTA0CCR0 = TA0CCR0;
    contextSaveCtl = *(group->inputCapsioctlRegister);
    /*
     *  TimerA0 is the measurement timer and counts the number of relaxation
     *  oscillation cycles of the element which is connected to TA0CLK.
     *  TimerA0 is in continuous mode.  TA0CCR0 is configured as a capture
     *  register and will be triggered as a SW capture event.
     */
    TA0CTL = TASSEL_3+MC_2;
    TA0CCTL0 = CM_3+CCIS_2+CAP+SCS;
    /*
     *  The RTC is the gate (measurement interval) timer.  The number of
     *  oscillations counted, by TimerA0, within the gate interval represents
     *  the measured capacitance.
     */
    for (i = 0; i<(group->numElements); i++)
    {
	    /* Enable Capacitive Touch IO oscillation */
	    *(group->inputCapsioctlRegister)
	            = ((group->arrayPtr[i])->inputBits)+CAPSIOEN;
	    TA0CTL |= TACLR;        // Clear Timer_A0 measurement timer
	    TA0CTL &= ~TAIFG;       // Clear the overflow flag
        /*
         *  The measGateSource represents the gate source for the RTC, which
         *  can be sourced from SMCLK, VLOCLK or X_CLK. The
         *  accumulationCycles represents the RTCMOD interval value.
         *
         */
        RTCCTL = RTCSR;
        if(__even_in_range(RTCIV,RTCIV_RTCIF));
        RTCMOD = group->accumulationCycles;
        RTCCTL |= group->measGateSource | RTCSR | group->sourceScale | RTCIE;
        /*
         *  The interrupt handler is defined in RTC_VECTOR, which simply clears
         *  the low power mode bits in the Status Register before returning
         *  from the ISR.
         */
        if(group->measGateSource == GATE_RTC_SMCLK)
        {
            __bis_SR_register(LPM0_bits+GIE); //Enable the GIE and wait for ISR
        }
        else
        {
	        __bis_SR_register(LPM3_bits+GIE);
        }
		
		TA0CCTL0 ^= CCIS0;  // Create SW capture of TA0R into TA0CCR0
        RTCCTL = RTCSR;
		
        if(TA0CTL & TAIFG)
        {
            /*
             *  If a rollover in the timer has occurred then set counts to
             *  0.  This will prevent erroneous data from entering the baseline
             *  tracking algorithm.
             */
            counts[i] = 0;
        }
        else
        {
            counts[i] = TA0CCR0;  // Save result
        }
    }  // End For Loop
    /* Context restore GIE within Status Register and registers used. */
    *(group->inputCapsioctlRegister) = contextSaveCtl;
    if(!(contextSaveSR & GIE))
    {
        __bic_SR_register(GIE);
    }
    RTCCTL = contextSaveRTCCTL;
    TA0CTL = contextSaveTA0CTL;
    TA0CCTL0 = contextSaveTA0CCTL0;
    TA0CCR0 = contextSaveTA0CCR0;
}
#endif

/*!
 *  @} 
 */

/*!
 *  @defgroup ISR_GROUP ISR Definitions
 *  @ingroup CTS_HAL
 */
 
#ifdef WDT_GATE
/*!
 *  ======== watchdog_timer ========
 *  @ingroup ISR_GROUP
 *  @brief  WDT_ISR
 *
 *          This ISR clears the LPM bits found in the Status Register (SR/R2).
 * 
 *  @param none
 *  @return none
 */
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer(void)
{
    __bic_SR_register_on_exit(LPM3_bits);           // Exit LPM3 on reti
}
#endif

#ifdef TIMER0A0_GATE
/*!
 *  ======== TIMER0_A0_ISR ========
 *  @ingroup ISR_GROUP
 *  @brief  TIMER0_A0_ISR
 *
 *          This ISR clears the LPM bits found in the Status Register (SR/R2).
 * 
 *  @param none
 *  @return none
 */
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
    __bic_SR_register_on_exit(LPM3_bits);           // Exit LPM3 on reti
}
#endif

#ifdef TIMER1A0_GATE
/*!
 *  ======== TIMER1_A0_ISR ========
 *  @ingroup ISR_GROUP
 *  @brief  TIMER1_A0_ISR
 *
 *          This ISR clears the LPM bits found in the Status Register (SR/R2).
 * 
 *  @param none
 *  @return none
 */
#pragma vector=TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)

{
    __bic_SR_register_on_exit(LPM3_bits);           // Exit LPM3 on reti
}
#endif

#ifdef TIMER2A0_GATE
/*!
 *  ======== TIMER2_A0_ISR ========
 *  @ingroup ISR_GROUP
 *  @brief  TIMER2_A0_ISR
 *
 *          This ISR clears the LPM bits found in the Status Register (SR/R2).
 * 
 *  @param none
 *  @return none
 */
#pragma vector=TIMER2_A0_VECTOR
__interrupt void TIMER2_A0_ISR(void)
{
    __bic_SR_register_on_exit(LPM3_bits);           // Exit LPM3 on reti
}
#endif

#ifdef TIMER3A0_GATE
/*!
 *  ======== TIMER3_A0_ISR ========
 *  @ingroup ISR_GROUP
 *  @brief  TIMER3_A0_ISR
 *
 *          This ISR clears the LPM bits found in the Status Register (SR/R2).
 * 
 *  @param none
 *  @return none
 */
#pragma vector=TIMER3_A0_VECTOR
__interrupt void TIMER3_A0_ISR(void)

{
    __bic_SR_register_on_exit(LPM3_bits);           // Exit LPM3 on reti
}
#endif

#ifdef TIMERB0_GATE
/*!
 *  ======== TIMER0_B0_ISR ========
 *  @ingroup ISR_GROUP
 *  @brief  TIMER0_B0_ISR
 *
 *          This ISR clears the LPM bits found in the Status Register (SR/R2).
 *
 *  @param none
 *  @return none
 */
#pragma vector=TIMERB0_VECTOR
__interrupt void TIMERB0_ISR(void)

{
    __bic_SR_register_on_exit(LPM3_bits);           // Exit LPM3 on reti
}
#endif

#ifdef RTC_GATE
/*!
 *  ======== RTC ========
 *  @ingroup ISR_GROUP
 *  @brief  RTC_ISR
 *
 *          This ISR clears the LPM bits found in the Status Register (SR/R2).
 *
 *  @param none
 *  @return none
 */
#pragma vector=RTC_VECTOR
__interrupt void rtc_timer(void)
{
	if(__even_in_range(RTCIV,RTCIV_RTCIF) == 0x02)
	{
            __bic_SR_register_on_exit(LPM3_bits);           // Exit LPM3 on reti
	}
}
#endif
