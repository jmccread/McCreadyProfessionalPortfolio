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
// usci_a_uart.h - Driver for the USCI_A_UART Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_USCI_A_UART_H__
#define __MSP430WARE_USCI_A_UART_H__

#include "inc/hw_memmap.h"
#include "inc/booltype.h"


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
// The following values are the sync characters possible
//
//*****************************************************************************
#define DEFAULT_SYNC                                                       0x00
#define USCI_A_UART_AUTOMATICBAUDRATE_SYNC                                 0x55

///////////////////////////////////////////////////////////////////////////////
//Deprecated
///////////////////////////////////////////////////////////////////////////////
#define UARTBREAK_DETECT                                                  UCBRK

//*****************************************************************************
//
// The following are values that can be passed to the parity parameter for
// functions: USCI_A_UART_initAdvance().
//
//*****************************************************************************
#define USCI_A_UART_NO_PARITY                                              0x00
#define USCI_A_UART_ODD_PARITY                                             0x01
#define USCI_A_UART_EVEN_PARITY                                            0x02

//*****************************************************************************
//
// The following are values that can be passed to the msborLsbFirst parameter
// for functions: USCI_A_UART_initAdvance().
//
//*****************************************************************************
#define USCI_A_UART_MSB_FIRST                                             UCMSB
#define USCI_A_UART_LSB_FIRST                                              0x00

//*****************************************************************************
//
// The following are values that can be passed to the uartMode parameter for
// functions: USCI_A_UART_initAdvance().
//
//*****************************************************************************
#define USCI_A_UART_MODE                                               UCMODE_0
#define USCI_A_UART_IDLE_LINE_MULTI_PROCESSOR_MODE                     UCMODE_1
#define USCI_A_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE                   UCMODE_2
#define USCI_A_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE                  UCMODE_3

//*****************************************************************************
//
// The following are values that can be passed to the selectClockSource
// parameter for functions: USCI_A_UART_initAdvance().
//
//*****************************************************************************
#define USCI_A_UART_CLOCKSOURCE_SMCLK                             UCSSEL__SMCLK
#define USCI_A_UART_CLOCKSOURCE_ACLK                               UCSSEL__ACLK
/*#define USCI_A_UART_CLOCKSOURCE_SMCLK                              0x80
#define USCI_A_UART_CLOCKSOURCE_ACLK                               0x40*/

//*****************************************************************************
//
// The following are values that can be passed to the numberofStopBits
// parameter for functions: USCI_A_UART_initAdvance().
//
//*****************************************************************************
#define USCI_A_UART_ONE_STOP_BIT                                           0x00
#define USCI_A_UART_TWO_STOP_BITS                                         UCSPB

//*****************************************************************************
//
// The following are values that can be passed to the overSampling parameter
// for functions: USCI_A_UART_initAdvance().
//
//*****************************************************************************
#define USCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION                       0x01
#define USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION                      0x00

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: USCI_A_UART_enableInterrupt(), and
// USCI_A_UART_disableInterrupt().
//
//*****************************************************************************
#define USCI_A_UART_RECEIVE_INTERRUPT                                    UCRXIE
#define USCI_A_UART_TRANSMIT_INTERRUPT                                   UCTXIE
#define USCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT                     UCRXEIE
#define USCI_A_UART_BREAKCHAR_INTERRUPT                                 UCBRKIE
/*#define USCI_A_UART_RECEIVE_INTERRUPT                                    0x0001
#define USCI_A_UART_TRANSMIT_INTERRUPT                                   0x0002
#define USCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT                     0x20
#define USCI_A_UART_BREAKCHAR_INTERRUPT                                 0x10*/

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: USCI_A_UART_getInterruptStatus(), and
// USCI_A_UART_clearInterruptFlag() as well as returned by the
// USCI_A_UART_getInterruptStatus() function.
//
//*****************************************************************************
#define USCI_A_UART_RECEIVE_INTERRUPT_FLAG                              0x0001
#define USCI_A_UART_TRANSMIT_INTERRUPT_FLAG                             0x0002

//*****************************************************************************
//
// The following are values that can be passed to the mask parameter for
// functions: USCI_A_UART_queryStatusFlags() as well as returned by the
// USCI_A_UART_queryStatusFlags() function.
//
//*****************************************************************************
#define USCI_A_UART_LISTEN_ENABLE                                      UCLISTEN
#define USCI_A_UART_FRAMING_ERROR                                          UCFE
#define USCI_A_UART_OVERRUN_ERROR                                          UCOE
#define USCI_A_UART_PARITY_ERROR                                           UCPE
#define USCI_A_UART_BREAK_DETECT                                          UCBRK
#define USCI_A_UART_RECEIVE_ERROR                                       UCRXERR
#define USCI_A_UART_ADDRESS_RECEIVED                                     UCADDR
#define USCI_A_UART_IDLELINE                                             UCIDLE
#define USCI_A_UART_BUSY                                                 UCBUSY

#ifndef GETCHAR_PUTCHAR_RETURN_ZERO
#define  EOF_RETURN_CODE            0
#else
#define  EOF_RETURN_CODE            0
#endif

#ifndef UART_MAX_RX_BUFF_SIZE
#define UART_MAX_RX_BUFF_SIZE             255
#endif

#ifndef UART_MAX_TX_BUFF_SIZE
#define UART_MAX_TX_BUFF_SIZE             255
#endif

// UART Bus struct
typedef struct usci_a_uart_td {
	uint32_t  baseAddress;
	uint32_t  UCAxIE;
	uint32_t  UCAxIFG;
	uint32_t  UCAxCTL1;
	uint32_t  UCAxCTL0;
	uint32_t  UCAxBR0;
	uint32_t  UCAxBR1;
	uint32_t  UCAxMCTL;
	uint32_t  UCAxSTAT;
	uint32_t  UCAxTXBUF;
	uint32_t  UCAxRXBUF;
} usci_a_uart_t;


//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************

extern void USCI_A_UART_initregs(usci_a_uart_t *in_ptr);

extern uint8_t USCI_A_UART_init( usci_a_uart_t *in_ptr,
                          uint32_t	baseAddressin,
                          unsigned char selectClockSource,
                          unsigned long clockSourceFrequency,
                          unsigned long desiredUartBaudRate,
                          unsigned char parity,
                          unsigned char msborLsbFirst,
                          unsigned char numberofStopBits,
                          unsigned char uartMode,
                          unsigned short overSampling
                          );

extern void USCI_A_UART_initAdvance(usci_a_uart_t *in_ptr,
                                    uint8_t selectClockSource,
                                    uint16_t clockPrescalar,
                                    uint8_t firstModReg,
                                    uint8_t secondModReg,
                                    uint8_t parity,
                                    uint8_t msborLsbFirst,
                                    uint8_t numberofStopBits,
                                    uint8_t uartMode,
                                    uint16_t overSampling);

extern void USCI_A_UART_transmitData(usci_a_uart_t *in_ptr, uint8_t transmitData);

extern uint8_t USCI_A_UART_receiveData(usci_a_uart_t *in_ptr);

extern void USCI_A_UART_enableInterrupt(usci_a_uart_t *in_ptr, uint8_t mask);

extern void USCI_A_UART_disableInterrupt(usci_a_uart_t *in_ptr, uint8_t mask);

extern uint8_t USCI_A_UART_getInterruptStatus(usci_a_uart_t *in_ptr, uint8_t mask);

extern void USCI_A_UART_clearInterruptFlag(usci_a_uart_t *in_ptr, uint8_t mask);

extern void USCI_A_UART_enable(usci_a_uart_t *in_ptr);

extern void USCI_A_UART_disable(usci_a_uart_t *in_ptr);


/*extern uint16_t USCI_A_UART_rxbuff_count(usci_a_uart_t *in_ptr);
extern uint16_t USCI_A_UART_txbuff_free(usci_a_uart_t *in_ptr);
extern uint8_t  USCI_A_UART_getchar(usci_a_uart_t *in_ptr);
extern uint16_t USCI_A_UART_putchars(usci_a_uart_t *in_ptr, uint8_t *in, uint16_t inLength);
extern uint16_t USCI_A_UART_puts(usci_a_uart_t *in_ptr, const uint8_t *c);

extern uint8_t USCI_A_UART_outchar(usci_a_uart_t *in_ptr);
extern void    USCI_A_UART_inchar(usci_a_uart_t *in_ptr);*/


extern uint8_t USCI_A_UART_queryStatusFlags(usci_a_uart_t *in_ptr,
                                            uint8_t mask);

extern void USCI_A_UART_setDormant(usci_a_uart_t *in_ptr);

extern void USCI_A_UART_resetDormant(usci_a_uart_t *in_ptr);

extern void USCI_A_UART_transmitAddress(usci_a_uart_t *in_ptr,
                                        uint8_t transmitAddress);

extern void USCI_A_UART_transmitBreak(usci_a_uart_t *in_ptr);

extern uint32_t USCI_A_UART_getReceiveBufferAddressForDMA(usci_a_uart_t *in_ptr);

extern uint32_t USCI_A_UART_getTransmitBufferAddressForDMA(usci_a_uart_t *in_ptr);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __MSP430WARE_USCI_A_UART_H__
