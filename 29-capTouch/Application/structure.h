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
/*
 * structure.h
 *
 * RO_PINOSC_TA0_WDTp_One_Button example
 * Touch the middle element to turn on/off the center button LED
 * RO method capactiance measurement using PinOsc IO, TimerA0, and WDT+
 *
 * This file contains both application dependent and independent pieces.  The 
 * dependent piece must be updated along with structure.c, and carries the
 * naming conventions found in structure.c to the rest of the library.
 */
//******************************************************************************
// The following elements need to be configured by the user.
//******************************************************************************
#ifndef CTS_STRUCTURE_H_
#define CTS_STRUCTURE_H_

#include "msp430.h"
#include <stdint.h>

/* Public Globals */

// Middle Element on the LaunchPad Capacitive Touch BoosterPack
extern const struct Element middle_element;

// One Button Sensor
extern const struct Sensor one_button;    

//****** RAM ALLOCATION ********************************************************
// TOTAL_NUMBER_OF_ELEMENTS represents the total number of elements used, even if 
// they are going to be segmented into seperate groups.  This defines the 
// RAM allocation for the baseline tracking.  If only the TI_CAPT_Raw function
// is used, then this definition should be removed to conserve RAM space.
#define TOTAL_NUMBER_OF_ELEMENTS 1
// If the RAM_FOR_FLASH definition is removed, then the appropriate HEAP size 
// must be allocated. 2 bytes * MAXIMUM_NUMBER_OF_ELEMENTS_PER_SENSOR + 2 bytes
// of overhead.
#define RAM_FOR_FLASH
//****** Structure Array Definition ********************************************
// This defines the array size in the sensor strucure.  In the event that 
// RAM_FOR_FLASH is defined, then this also defines the amount of RAM space
// allocated (global variable) for computations.
#define MAXIMUM_NUMBER_OF_ELEMENTS_PER_SENSOR  1
//****** Choosing a  Measurement Method ****************************************
// These variables are references to the definitions found in structure.c and
// must be generated per the application.
// possible values for the method field

// OSCILLATOR DEFINITIONS
//#define RO_COMPAp_TA0_WDTp  		64
#define RO_PINOSC_TA0_WDTp  		65
//#define RO_PINOSC_TA0       		66
//#define RO_COMPAp_TA1_WDTp  		67
//#define RO_COMPB_TA0_WDTA			68
//#define RO_COMPB_TA1_WDTA         69
//#define RO_COMPB_TB0_WDTA         70
//#define RO_COMPB_TA1_TA0          71
//#define RO_PINOSC_TA0_TA1         72
//#define RO_CSIO_TA2_WDTA			73
//#define RO_CSIO_TA2_TA3           74
//#define RO_PINOSC_TA1_WDTp  		75
//#define RO_PINOSC_TA1_TB0			76
//#define RO_CSIO_TA0_WDTA          77
//#define RO_CSIO_TA0_TA1           78
//#define RO_CSIO_TA0_RTC           79

// RC DEFINITIONS
//#define RC_PAIR_TA0       		01
         
// FAST RO DEFINITIONS
//#define fRO_CSIO_TA2_TA3          23
//#define fRO_PINOSC_TA0_TA1        24
//#define fRO_PINOSC_TA0_SW         25
//#define fRO_COMPB_TA0_SW          26
//#define fRO_COMPB_TA1_SW          27
//#define fRO_COMPAp_TA0_SW         28
//#define fRO_COMPAp_SW_TA0         29
//#define fRO_COMPAp_TA1_SW         30
//#define fRO_COMPB_TA1_TA0         31
//#define fRO_PINOSC_TA1_TA0		32
//#define fRO_PINOSC_TA1_TB0		33
//#define fRO_CSIO_TA0_SW           34
//#define fRO_CSIO_TA0_TA1          35

//****** WHEEL and SLIDER ******************************************************
// Are wheel or slider representations used?
//#define SLIDER
//#define ILLEGAL_SLIDER_WHEEL_POSITION		0xFFFF
//#define WHEEL

//******************************************************************************
// End of user configuration section.
//******************************************************************************
//******************************************************************************
//******************************************************************************

//possible timer source clock dividers, different from clock module dividers
#define TIMER_TxCLK 	   0x0000  // TxSSEL
#define TIMER_ACLK  	   0x0100
#define TIMER_SMCLK 	   0x0200
#define TIMER_INCLK 	   0x0300

#define TIMER_SOURCE_DIV_0 0x0000  // ID_0, IDX_0
#define TIMER_SOURCE_DIV_1 0x0040
#define TIMER_SOURCE_DIV_2 0x0080
#define TIMER_SOURCE_DIV_3 0x00C0

#define GATE_WDT_ACLK      0x0004
#define GATE_WDT_SMCLK     0x0000
#define GATE_WDTp_ACLK     0x0004
#define GATE_WDTp_SMCLK    0x0000

#define WDTp_GATE_32768    0x0000  // watchdog source/32768
#define WDTp_GATE_8192     0x0001  // watchdog source/8192
#define WDTp_GATE_512      0x0002  // watchdog source/512
#define WDTp_GATE_64       0x0003  // watchdog source/64

#define GATE_WDTA_SMCLK    0x0000
#define GATE_WDTA_ACLK     0x0020
#define GATE_WDTA_VLO      0x0040
#define GATE_WDTA_XCLK     0x0060

#define WDTA_GATE_2G       0x0000  // watchdog source/2G
#define WDTA_GATE_128M     0x0001  // watchdog source/128M
#define WDTA_GATE_8192K    0x0002  // watchdog source/8192K
#define WDTA_GATE_512K     0x0003  // watchdog source/512K
#define WDTA_GATE_32768    0x0004  // watchdog source/32768
#define WDTA_GATE_8192     0x0005  // watchdog source/8192
#define WDTA_GATE_512      0x0006  // watchdog source/512
#define WDTA_GATE_64       0x0007  // watchdog source/64

#define GATE_RTC_SMCLK     0x1000
#define GATE_RTC_VLO       0x3000
#define GATE_RTC_XCLK      0x2060

#define RTC_DIV_1          0x0000  //RTC Pre-scaler
#define RTC_DIV_10         0x0100
#define RTC_DIV_100        0x0200
#define RTC_DIV_1000       0x0300
#define RTC_DIV_16         0x0400
#define RTC_DIV_64         0x0500
#define RTC_DIV_256        0x0600
#define RTC_DIV_1024       0x0700


// The below variables are used to excluded portions of code not needed by
// the method chosen by the user. Uncomment the type used prior to compilation.
// Multiple types can be chosen as needed.
// What Method(s) are used in this application?

#ifdef RO_COMPAp_TA0_WDTp
    #define RO_TYPE
    #define RO_COMPAp_TYPE
    #define WDT_GATE
    #define HAL_DEFINITION
    //what devices have Pxsel2 ??
    // msp430f2112, 2122, 2132
    // msp430G2112, G2212, G2312, G2412, G2152, G2252, G2352, G2452
    // SEL2REGISTER
    #ifdef __MSP430F2112
      #define SEL2REGISTER
    #endif 
    #ifdef __MSP430F2122
      #define SEL2REGISTER
    #endif 
    #ifdef __MSP430F2132
      #define SEL2REGISTER
    #endif 
    #ifdef __MSP430G2112
      #define SEL2REGISTER
    #endif
    #ifdef __MSP430G2212
      #define SEL2REGISTER
    #endif      
    #ifdef __MSP430G2312
      #define SEL2REGISTER
    #endif  
    #ifdef __MSP430G2412
      #define SEL2REGISTER
    #endif  
    #ifdef __MSP430G2152
      #define SEL2REGISTER
    #endif 
    #ifdef __MSP430G2252
      #define SEL2REGISTER
    #endif 
    #ifdef __MSP430G2352
      #define SEL2REGISTER
    #endif 
    #ifdef __MSP430G2452
      #define SEL2REGISTER
    #endif 
#endif

#ifdef RO_PINOSC_TA0_WDTp
    #define RO_TYPE
    #define RO_PINOSC_TYPE
    #define WDT_GATE
    #define HAL_DEFINITION
#endif

#ifdef RO_PINOSC_TA0
    #define RO_TYPE
    #define RO_PINOSC_TYPE
    #define ACCUMULATE_TYPE
    #define HAL_DEFINITION
#endif

#ifdef RO_COMPAp_TA1_WDTp
    #define RO_TYPE
    #define RO_COMPAp_TYPE
    #define WDT_GATE
    #define HAL_DEFINITION
#endif

#ifdef RO_COMPB_TA0_WDTA
    #define RO_TYPE
    #define RO_COMPB_TYPE
    #define WDT_GATE
    #define HAL_DEFINITION
#endif

#ifdef RO_COMPB_TA1_WDTA
    #define RO_TYPE
    #define RO_COMPB_TYPE
    #define WDT_GATE
    #define HAL_DEFINITION
#endif

#ifdef RO_COMPB_TB0_WDTA
    #define RO_TYPE
    #define RO_COMPB_TYPE
    #define WDT_GATE
    #define HAL_DEFINITION
#endif

#ifdef RC_PAIR_TA0
    #define RC_TYPE
    #define RC_PAIR_TYPE
    #define ACCUMULATE_TYPE
    #define HAL_DEFINITION
#endif

#ifdef fRO_PINOSC_TA0_SW
	#define RO_TYPE
    #define RO_PINOSC_TYPE
    #define TIMER_SCALE
	#define HAL_DEFINITION
#endif

#ifdef fRO_COMPB_TA0_SW
    #define RO_TYPE
    #define RO_COMPB_TYPE
    #define TIMER_SCALE
	#define HAL_DEFINITION
#endif

#ifdef fRO_COMPB_TA1_SW
    #define RO_TYPE
    #define RO_COMPB_TYPE
    #define TIMER_SCALE
	#define HAL_DEFINITION
#endif

#ifdef fRO_COMPAp_TA0_SW
    #define RO_TYPE
    #define RO_COMPAp_TYPE
    #define TIMER_SCALE
	#define HAL_DEFINITION
#endif

#ifdef fRO_COMPAp_TA1_SW
    #define RO_TYPE
    #define RO_COMPAp_TYPE
    #define TIMER_SCALE
    #define HAL_DEFINITION
#endif

#ifdef fRO_COMPAp_SW_TA0
    #define RO_TYPE
    #define RO_COMPAp_TYPE
	#define TIMER_SCALE
    #define HAL_DEFINITION
#endif

#ifdef RO_COMPB_TA1_TA0
    #define RO_TYPE
    #define RO_COMPB_TYPE
    #define TIMER_SCALE
    #define TIMER0A0_GATE
    #define HAL_DEFINITION
#endif

#ifdef fRO_COMPB_TA1_TA0
    #define RO_TYPE
    #define RO_COMPB_TYPE
    #define TIMER_SCALE
    #define TIMER1A0_GATE
    #define HAL_DEFINITION
#endif

#ifdef RO_PINOSC_TA0_TA1
    #define RO_TYPE
    #define RO_PINOSC_TYPE
    #define TIMER_SCALE
    #define TIMER1A0_GATE
    #define HAL_DEFINITION
#endif

#ifdef fRO_PINOSC_TA0_TA1
    #define RO_TYPE
    #define RO_PINOSC_TYPE
    #define TIMER_SCALE
    #define TIMER0A0_GATE
    #define HAL_DEFINITION
#endif

#ifdef RO_CSIO_TA2_WDTA
    #define RO_TYPE
    #define RO_CSIO_TYPE
    #define WDT_GATE
    #define HAL_DEFINITION
#endif

#ifdef RO_CSIO_TA2_TA3
    #define RO_TYPE
    #define RO_CSIO_TYPE
    #define TIMER_SCALE
    #define TIMER3A0_GATE
    #define HAL_DEFINITION
#endif

#ifdef fRO_CSIO_TA2_TA3
    #define RO_TYPE
    #define RO_CSIO_TYPE
    #define TIMER_SCALE
    #define TIMER2A0_GATE
    #define HAL_DEFINITION
#endif

#ifdef RO_PINOSC_TA1_WDTp
    #define RO_TYPE
    #define RO_PINOSC_TYPE
    #define WDT_GATE
    #define HAL_DEFINITION
#endif

#ifdef RO_PINOSC_TA1_TB0
    #define RO_TYPE
    #define RO_PINOSC_TYPE
    #define TIMER_SCALE
    #define TIMERB0_GATE
    #define HAL_DEFINITION
#endif

#ifdef fRO_PINOSC_TA1_TA0
    #define RO_TYPE
    #define RO_PINOSC_TYPE
    #define TIMER_SCALE
    #define TIMER1A0_GATE
    #define HAL_DEFINITION
#endif

#ifdef fRO_PINOSC_TA1_TB0
	#define RO_TYPE
	#define RO_PINOSC_TYPE
	#define TIMER_SCALE
	#define TIMER1A0_GATE
	#define HAL_DEFINITION
#endif

#ifdef RO_CSIO_TA0_WDTA
    #define RO_TYPE
    #define RO_CSIO_TYPE
    #define WDT_GATE
    #define HAL_DEFINITION
#endif

#ifdef RO_CSIO_TA0_TA1
    #define RO_TYPE
    #define RO_CSIO_TYPE
    #define TIMER_SCALE
    #define TIMER1A0_GATE
    #define HAL_DEFINITION
#endif

#ifdef fRO_CSIO_TA0_TA1
    #define RO_TYPE
    #define RO_CSIO_TYPE
    #define TIMER_SCALE
    #define TIMER0A0_GATE
    #define HAL_DEFINITION
#endif

#ifdef fRO_CSIO_TA0_SW
	#define RO_TYPE
    #define RO_CSIO_TYPE
    #define TIMER_SCALE
	#define HAL_DEFINITION
#endif

#ifdef RO_CSIO_TA0_RTC
    #define RO_TYPE
    #define RO_CSIO_TYPE
    #define TIMER_SCALE
    #define RTC_GATE
	#define HAL_DEFINITION
#endif



#ifdef SLIDER
	#define SLIDER_WHEEL
#endif

#ifdef WHEEL
	#define SLIDER_WHEEL
#endif

#define RO_MASK         0xC0        // 1100 0000
#define RC_FRO_MASK     0x3F        // 0011 1111

/*
 *  The element structure identifies port or comparator input definitions for
 *  each element.
 */
struct Element{

#ifdef RO_PINOSC_TYPE
// These register address definitions are needed for each sensor only
// when using the PinOsc method
  uint8_t *inputPxselRegister;    // PinOsc: port selection address
  uint8_t *inputPxsel2Register;   // PinOsc: port selection 2 address  
#endif
  
#ifdef RC_PAIR_TYPE
// these fields are specific to the RC type. 
  uint8_t *inputPxoutRegister;    // RC: port output address: PxOUT
  volatile uint8_t *inputPxinRegister;     // RC: port input address: PxIN
  uint8_t *inputPxdirRegister;    // RC+PinOsc: port direction address
  uint8_t *referencePxoutRegister;// RC: port output address: PxOUT
  uint8_t *referencePxdirRegister;// RC: port direction address: PxDIR
  uint8_t referenceBits;           // RC: port bit definition
#endif
 		     
  uint16_t inputBits;                 // Comp_RO+FastRO+RC+PinOsc: bit 
                                      // definition
                                      //
                                      // for comparator input bit 
                                      // location in CACTL2 or CBCTL0
                                      
  uint16_t threshold;                   // specific threshold for each button
  uint16_t maxResponse;                 // Special Case: Slider max counts
};

/*
 *  The sensor structure identifies HAL and timing definitions for
 *  each sensor.
 */

struct Sensor{
  // the method acts as the switch to determine which HAL is called
  uint8_t halDefinition;           // COMPARATOR_TYPE (RO), RC, etc
                                   // RO_COMPA, RO_COMPB, RO_PINOSC
                                   // RC_GPIO, RC_COMPA, RC_COMPB
                                   // FAST_SCAN_RO
#ifdef RO_CSIO_TYPE
/*
 *  This register address definition is needed to indicate which CSIOxCTL
 *  register is associated with the Timer identified in the HAL.
 */
  uint16_t *inputCapsioctlRegister;
#endif

  uint8_t numElements;             // number of elements within group
  uint8_t baseOffset;              // the offset within the global 
                                   // base_cnt array

  struct Element const *arrayPtr[MAXIMUM_NUMBER_OF_ELEMENTS_PER_SENSOR];    
                                   // an array of pointers

//******************************************************************************
// Reference structure definitions for comparator types, for the RC method the 
// reference is defined within the element.
  
#ifdef RO_COMPAp_TYPE
  uint8_t * refPxoutRegister;      // RO+FastRO: port output address
  uint8_t * refPxdirRegister;      // RO+FastRO: port direction address
  uint8_t refBits;                 // RO+FastRO: port bit definition
  
  uint8_t * txclkDirRegister;      // PxDIR 
  uint8_t * txclkSelRegister;      // PxSEL
  uint8_t txclkBits;               // Bit field for register
  
  uint8_t *caoutDirRegister;      // PxDIR
  uint8_t *caoutSelRegister;      // PxSEL
  uint8_t caoutBits;               // Bit field for register
  
  // This is only applicable to the RO_COMPAp_TYPE
#ifdef SEL2REGISTER
  uint8_t *caoutSel2Register;
  uint8_t *txclkSel2Register;
#endif
  
  uint8_t refCactl2Bits;          // RO: CACTL2 input definition, 
                                          // CA0 (P2CA0),CA1(P2CA4),
                                          // CA2(P2CA0+P2CA4)
  uint8_t capdBits;
#endif 

#ifdef RO_COMPB_TYPE
  uint8_t *cboutTAxDirRegister;  // CBOUT_TA0CLK 
  uint8_t *cboutTAxSelRegister;  // CBOUT_TA0CLK
  uint8_t cboutTAxBits;           // Bit field for register  
  uint16_t cbpdBits;
#endif

//*****************************************************************************
// Timer definitions
//  The basic premise is to count a number of clock cycles within a time
//  period, where either the clock source or the timer period is a function
//  of the element capacitance.
// 
// RC Method:
//          Period: accumulationCycles * charge and discharge time of RC 
//          circuit where C is capacitive touch element
//
//          clock source: measGateSource/sourceScale
// RO Method:
//          Period: accumulationCycles*measGateSource/sourceScale 
//                  (with WDT sourceScale = 1, accumulationCycles is WDT control
//                   register settings)
//
//          clock source: relaxation oscillator where freq is a function of C
//
// fRO Method:
//          Period: accumulationCycles * 1/freq, freq is a function of C
//
//          clock source: measGateSource/sourceScale
     
  uint16_t measGateSource;         // RC+FastRO: measurement timer source,
                                   // {ACLK, TACLK, SMCLK}
                                   // Comp_RO+PinOsc: gate timer source, 
                                   // {ACLK, TACLK, SMCLK}
#ifdef TIMER_SCALE
  uint16_t sourceScale;            // RO+FastRO: gate timer,
                                   // TA/TB, scale: 1,1/2,1/4,1/8
                                   // RC+FastRO: measurement timer, TA/TB/TD
                                   // scale: 16, 8, 4, 2, 1, ?, ?, 1/8  
                                   // Not used for WDTp/WDTA
#endif
    
  uint16_t accumulationCycles;
  
//*****************************************************************************
// Other definitions

#ifdef SLIDER_WHEEL  
  uint8_t points;                   // Special Case: Number of points
                                    // along slider or wheel  
  uint8_t sensorThreshold;
#endif                               

};

/*
 *  The GCC language extension within CCS is needed, otherwise a warning will
 *  be generated during compilation when no problems exist or an error will be
 *  generated (instead of a warning) when a problem does exist.
 */
#ifndef TOTAL_NUMBER_OF_ELEMENTS
 #warning "WARNING: TOTAL_NUMBER_OF_ELEMENTS is not defined in structure.h. Only TI_CAPT_RAW function is enabled."
#endif

#ifndef RAM_FOR_FLASH
 #warning "WARNING: The HEAP must be set appropriately.  Please refer to SLAA490 for details."
#endif

#ifndef HAL_DEFINITION
 #warning "WARNING: At least one HAL definition must be made in structure.h."
#endif

#endif /* CTS_STRUCTURE_H_ */
