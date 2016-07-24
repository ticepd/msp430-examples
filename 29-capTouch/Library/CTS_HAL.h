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
 *  @file   CTS_HAL.h
 *
 *  @brief       
 *
 *  @par    Project:
 *             MSP430 Capacitive Touch Library 
 *
 *  @par    Developed using:
 *             CCS Version : 5.2.1.00018, w/support for GCC extensions (--gcc)
 *  \n         IAR Version : 5.51.2 [Kickstart]
 *
 *  @author    C. Sterzik
 *  @author    T. Hwang
 *  @author    E. Fu
 *
 *  @version     1.4 addresses bugs in v1.3.
 *
 *  @par    Supported Hardware Configurations:
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
 *              - TI_CTS_RO_PINOSC_TA1_WDTp_HAL()
 *              - TI_CTS_RO_PINOSC_TA1_TB0_HAL()
 *              - TI_CTS_fRO_PINOSC_TA1_TA0_HAL()
 *              - TI_CTS_fRO_PINOSC_TA1_TB0_HAL()
 *           /n  (1.3)                                           
 *              - TI_CTS_RO_CSIO_TA0_TA1_HAL()
 *              - TI_CTS_fRO_CSIO_TA0_TA1_HAL()
 *              - TI_CTS_fRO_CSIO_TA0_SW_HAL()
 *              - TI_CTS_RO_CSIO_TA0_RTC_HAL()
 *              - TI_CTS_RO_CSIO_TA0_WDTA_HAL()
 *
 */

#ifndef CTS_HAL_H_
#define CTS_HAL_H_

#include "structure.h"

void TI_CTS_RO_CSIO_TA0_TA1_HAL(const struct Sensor *,uint16_t *);

void TI_CTS_fRO_CSIO_TA0_TA1_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_fRO_CSIO_TA0_SW_HAL(const struct Sensor *,uint16_t *);

void TI_CTS_RO_CSIO_TA0_RTC_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_RO_CSIO_TA0_WDTA_HAL(const struct Sensor *,uint16_t *);

void TI_CTS_RO_CSIO_TA2_WDTA_HAL(const struct Sensor *,uint16_t *);

void TI_CTS_RO_CSIO_TA2_TA3_HAL(const struct Sensor *,uint16_t *);

void TI_CTS_fRO_CSIO_TA2_TA3_HAL(const struct Sensor *,uint16_t *);

void TI_CTS_RO_COMPB_TB0_WDTA_HAL(const struct Sensor *,uint16_t *);

void TI_CTS_RO_PINOSC_TA0_TA1_HAL(const struct Sensor *,uint16_t *);

void TI_CTS_fRO_PINOSC_TA0_TA1_HAL(const struct Sensor *,uint16_t *);

void TI_CTS_RO_COMPAp_TA0_WDTp_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_fRO_COMPAp_TA0_SW_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_fRO_COMPAp_SW_TA0_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_RO_COMPAp_TA1_WDTp_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_fRO_COMPAp_TA1_SW_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_RC_PAIR_TA0_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_RO_PINOSC_TA0_WDTp_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_RO_PINOSC_TA0_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_fRO_PINOSC_TA0_SW_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_RO_COMPB_TA0_WDTA_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_fRO_COMPB_TA0_SW_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_RO_COMPB_TA1_WDTA_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_fRO_COMPB_TA1_SW_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_RO_COMPB_TA1_TA0_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_fRO_COMPB_TA1_TA0_HAL(const struct Sensor *,uint16_t *);

void TI_CTS_RO_PINOSC_TA1_WDTp_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_RO_PINOSC_TA1_TB0_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_fRO_PINOSC_TA1_TA0_HAL(const struct Sensor *, uint16_t *);

void TI_CTS_fRO_PINOSC_TA1_TB0_HAL(const struct Sensor *, uint16_t *);


#endif /* CTS_HAL_H_ */
