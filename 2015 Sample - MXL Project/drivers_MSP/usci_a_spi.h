/* --COPYRIGHT--,BSD
 * Copyright (c) 2013, Texas Instruments Incorporated
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
//*****************************************************************************
//
// usci_a_spi.h - Driver for the USCI_A_SPI Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_USCI_A_SPI_H__
#define __MSP430WARE_USCI_A_SPI_H__

#include "inc/hw_memmap.h"
#include "inc/booltype.h"

#ifdef __MSP430_HAS_USCI_Ax__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// The following are values that can be passed to the clockPhase parameter for
// functions: USCI_A_SPI_masterInit(), USCI_A_SPI_slaveInit(), and
// USCI_A_SPI_changeClockPhasePolarity().
//
//*****************************************************************************
#define USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT             0x00
#define USCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT           UCCKPH

//*****************************************************************************
//
// The following are values that can be passed to the msbFirst parameter for
// functions: USCI_A_SPI_masterInit(), and USCI_A_SPI_slaveInit().
//
//*****************************************************************************
#define USCI_A_SPI_MSB_FIRST                                              UCMSB
#define USCI_A_SPI_LSB_FIRST                                               0x00

//*****************************************************************************
//
// The following are values that can be passed to the clockPolarity parameter
// for functions: USCI_A_SPI_masterInit(), USCI_A_SPI_slaveInit(), and
// USCI_A_SPI_changeClockPhasePolarity().
//
//*****************************************************************************
#define USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH                         UCCKPL
#define USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW                            0x00

//*****************************************************************************
//
// The following are values that can be passed to the selectClockSource
// parameter for functions: USCI_A_SPI_masterInit().
//
//*****************************************************************************
#define USCI_A_SPI_CLOCKSOURCE_ACLK                                UCSSEL__ACLK
#define USCI_A_SPI_CLOCKSOURCE_SMCLK                              UCSSEL__SMCLK

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: USCI_A_SPI_enableInterrupt(), USCI_A_SPI_disableInterrupt(),
// USCI_A_SPI_getInterruptStatus(), and USCI_A_SPI_clearInterruptFlag() as well
// as returned by the USCI_A_SPI_getInterruptStatus() function.
//
//*****************************************************************************
#define USCI_A_SPI_TRANSMIT_INTERRUPT                                    UCTXIE
#define USCI_A_SPI_RECEIVE_INTERRUPT                                     UCRXIE

//*****************************************************************************
//
// The following are values that can be passed toThe following are values that
// can be returned by the USCI_A_SPI_isBusy() function.
//
//*****************************************************************************
#define USCI_A_SPI_BUSY                                                  UCBUSY
#define USCI_A_SPI_NOT_BUSY                                                0x00

// UART Bus struct
typedef struct usci_a_spi_td {
	uint32_t  baseAddress;
	uint32_t  UCAxIE;
	uint32_t  UCAxIFG;
	uint32_t  UCAxCTL1;
	uint32_t  UCAxCTL0;
	uint32_t  UCAxBRW;
	uint32_t  UCAxBR1;
	uint32_t  UCAxMCTL;
	uint32_t  UCAxSTAT;
	uint32_t  UCAxTXBUF;
	uint32_t  UCAxRXBUF;

} usci_a_spi_t;


//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************

extern void USCI_A_SPI_initregs(usci_a_spi_t *in_ptr);

extern bool_t USCI_A_SPI_masterInit(usci_a_spi_t *in_ptr,
uint32_t baseAddressin,
                                  uint8_t selectClockSource,
                                  uint32_t clockSourceFrequency,
                                  uint32_t desiredSpiClock,
                                  uint8_t msbFirst,
                                  uint8_t clockPhase,
                                  uint8_t clockPolarity);

extern void USCI_A_SPI_masterChangeClock(usci_a_spi_t *in_ptr,
                                         uint32_t clockSourceFrequency,
                                         uint32_t desiredSpiClock);

extern bool_t USCI_A_SPI_slaveInit(usci_a_spi_t *in_ptr,
uint32_t baseAddressin,
                                 uint8_t msbFirst,
                                 uint8_t clockPhase,
                                 uint8_t clockPolarity);

extern void USCI_A_SPI_changeClockPhasePolarity(usci_a_spi_t *in_ptr,
                                                uint8_t clockPhase,
                                                uint8_t clockPolarity);

extern void USCI_A_SPI_transmitData(usci_a_spi_t *in_ptr,
                                    uint8_t transmitData);

extern uint8_t USCI_A_SPI_receiveData(usci_a_spi_t *in_ptr);

extern void USCI_A_SPI_enableInterrupt(usci_a_spi_t *in_ptr,
                                       uint8_t mask);

extern void USCI_A_SPI_disableInterrupt(usci_a_spi_t *in_ptr,
                                        uint8_t mask);

extern uint8_t USCI_A_SPI_getInterruptStatus(usci_a_spi_t *in_ptr,
                                             uint8_t mask);

extern void USCI_A_SPI_clearInterruptFlag(usci_a_spi_t *in_ptr,
                                          uint8_t mask);

extern void USCI_A_SPI_enable(usci_a_spi_t *in_ptr);

extern void USCI_A_SPI_disable(usci_a_spi_t *in_ptr);

extern uint32_t USCI_A_SPI_getReceiveBufferAddressForDMA(usci_a_spi_t *in_ptr);

extern uint32_t USCI_A_SPI_getTransmitBufferAddressForDMA(usci_a_spi_t *in_ptr);

extern uint8_t USCI_A_SPI_isBusy(usci_a_spi_t *in_ptr);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_USCI_A_SPI_H__
