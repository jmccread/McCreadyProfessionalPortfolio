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
// usci_a_uart.c - Driver for the usci_a_uart Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup usci_a_uart_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/booltype.h"



#if  defined(__MSP430_HAS_USCI_Ax__) || defined(__MSP430_HAS_USCI_ABx__)
#include "usci_a_uart.h"
#include "uart_baudrate.h"

#if  defined(__msp430x26x)
void USCI_A_UART_initregs(usci_a_uart_t *in_ptr)
{
	switch(in_ptr->baseAddress)
	{	
		case USCI_A0_BASE:
		in_ptr->UCAxIE		=	IE2_;
		in_ptr->UCAxIFG		=	IFG2_;
		in_ptr->UCAxCTL1	=	UCA0CTL1_;
		in_ptr->UCAxCTL0	=	UCA0CTL0_;
		in_ptr->UCAxBR0		=	UCA0BR0_;
		in_ptr->UCAxBR1		= 	UCA0BR1_;
		in_ptr->UCAxMCTL	=	UCA0MCTL_;
		in_ptr->UCAxSTAT	=	UCA0STAT_;
		in_ptr->UCAxTXBUF	=	UCA0TXBUF_;
		in_ptr->UCAxRXBUF	=	UCA0RXBUF_;
		break;
		
		case USCI_A1_BASE:
		in_ptr->UCAxIE		=	UC1IE_;
		in_ptr->UCAxIFG		=	UC1IFG_;
		in_ptr->UCAxCTL1	=	UCA1CTL1_;
		in_ptr->UCAxCTL0	=	UCA1CTL0_;
		in_ptr->UCAxBR0		=	UCA1BR0_;
		in_ptr->UCAxBR1		= 	UCA1BR1_;
		in_ptr->UCAxMCTL	=	UCA1MCTL_;
		in_ptr->UCAxSTAT	=	UCA1STAT_;
		in_ptr->UCAxTXBUF	=	UCA1TXBUF_;
		in_ptr->UCAxRXBUF	=	UCA1RXBUF_;
		break;
		
		default: break;
	}
}
#else
void USCI_A_UART_initregs(usci_a_uart_t *in_ptr)
{

	in_ptr->UCAxIE		=	in_ptr->baseAddress + OFS_UCAxIE;
	in_ptr->UCAxIFG		=	in_ptr->baseAddress + OFS_UCAxIFG;
	in_ptr->UCAxCTL1	=	in_ptr->baseAddress + OFS_UCAxCTL1;
	in_ptr->UCAxCTL0	=	in_ptr->baseAddress + OFS_UCAxCTL0;
	in_ptr->UCAxBR0		=	in_ptr->baseAddress + OFS_UCAxBR0;
	in_ptr->UCAxBR1		= 	in_ptr->baseAddress + OFS_UCAxBR1;
	in_ptr->UCAxMCTL	=	in_ptr->baseAddress + OFS_UCAxMCTL;
	in_ptr->UCAxSTAT	=	in_ptr->baseAddress + OFS_UCAxSTAT;
	in_ptr->UCAxTXBUF	=	in_ptr->baseAddress + OFS_UCAxTXBUF;
	in_ptr->UCAxRXBUF	=	in_ptr->baseAddress + OFS_UCAxRXBUF;

}
#endif

uint8_t USCI_A_UART_init( usci_a_uart_t *in_ptr,
			  uint32_t	baseAddressin,
                          unsigned char selectClockSource,
                          unsigned long clockSourceFrequency,
                          unsigned long desiredUartBaudRate,
                          unsigned char parity,
                          unsigned char msborLsbFirst,
                          unsigned char numberofStopBits,
                          unsigned char uartMode,
                          unsigned short overSampling
                          )
{
        unsigned char UCAxBR0_value = 0x00;
        unsigned char UCAxBR1_value = 0x00;
        unsigned int UCAxMCTL_value = 0x00;

        in_ptr->baseAddress = baseAddressin;

        USCI_A_UART_initregs(in_ptr);

        //Disable the USCI Module
        HWREG8(in_ptr->UCAxCTL1) |= UCSWRST;

        //Clock source select
        HWREG8(in_ptr->UCAxCTL1) &= ~UCSSEL_3;
        HWREG8(in_ptr->UCAxCTL1) |= selectClockSource;

        //MSB, LSB select
        HWREG8(in_ptr->UCAxCTL0) &= ~UCMSB;
        HWREG8(in_ptr->UCAxCTL0) |= msborLsbFirst;


        //UCSPB = 0(1 stop bit) OR 1(2 stop bits)
        HWREG8(in_ptr->UCAxCTL0) &= ~UCSPB;
        HWREG8(in_ptr->UCAxCTL0) |= numberofStopBits;


        //Parity
        switch (parity) {
        case USCI_A_UART_NO_PARITY:
                //No Parity
                HWREG8(in_ptr->UCAxCTL0) &= ~UCPEN;
                break;
        case USCI_A_UART_ODD_PARITY:
                //Odd Parity
                HWREG8(in_ptr->UCAxCTL0) |= UCPEN;
                HWREG8(in_ptr->UCAxCTL0) &= ~UCPAR;
                break;
        case USCI_A_UART_EVEN_PARITY:
                //Even Parity
                HWREG8(in_ptr->UCAxCTL0) |= UCPEN;
                HWREG8(in_ptr->UCAxCTL0) |= UCPAR;
                break;
        }

        //Calculate Baud rate divider values for Modulation control registers
        if ( STATUS_FAIL == UARTBAUDRATE_calculateBaudDividers(clockSourceFrequency,
                                                               desiredUartBaudRate,
                                                               &UCAxBR0_value,
                                                               &UCAxBR1_value,
                                                               &UCAxMCTL_value,
                                                               overSampling
                                                               ))
                return STATUS_FAIL;

        //Modulation Control Registers
        HWREG8(in_ptr->UCAxBR0 ) = UCAxBR0_value;
        HWREG8(in_ptr->UCAxBR1) = UCAxBR1_value;
        HWREG8(in_ptr->UCAxMCTL) = UCAxMCTL_value;

        //Asynchronous mode & 8 bit character select & clear mode
        HWREG8(in_ptr->UCAxCTL0) &=  ~(UCSYNC +
                                                 UC7BIT +
                                                 UCMODE_3
                                                 );

        //Configure  UART mode.
        HWREG8(in_ptr->UCAxCTL0) |= uartMode;

        //Reset UCRXIE, UCBRKIE, UCDORM, UCTXADDR, UCTXBRK
        HWREG8(in_ptr->UCAxCTL1)  &= ~(UCRXEIE + UCBRKIE + UCDORM +
                                                 UCTXADDR + UCTXBRK
                                                 );

   
}

//*****************************************************************************
//
//! \brief Advanced initialization routine for the UART block. The values to be
//! written into the clockPrescalar, firstModReg, secondModReg and overSampling
//! parameters should be pre-computed and passed into the initialization
//! function.
//!
//! Upon successful initialization of the UART block, this function will have
//! initialized the module, but the UART block still remains disabled and must
//! be enabled with USCI_A_UART_enable(). To calculate values for
//! clockPrescalar, firstModReg, secondModReg and overSampling please use the
//! link below.
//!
//! http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//! \param selectClockSource selects Clock source.
//!        Valid values are:
//!        - \b USCI_A_UART_CLOCKSOURCE_SMCLK
//!        - \b USCI_A_UART_CLOCKSOURCE_ACLK
//! \param clockPrescalar is the value to be written into UCBRx bits
//! \param firstModReg is First modulation stage register setting. This value
//!        is a pre-calculated value which can be obtained from the Device
//!        Users Guide. This value is written into UCBRFx bits of UCAxMCTLW.
//! \param secondModReg is Second modulation stage register setting. This value
//!        is a pre-calculated value which can be obtained from the Device
//!        Users Guide. This value is written into UCBRSx bits of UCAxMCTLW.
//! \param parity is the desired parity.
//!        Valid values are:
//!        - \b USCI_A_UART_NO_PARITY [Default]
//!        - \b USCI_A_UART_ODD_PARITY
//!        - \b USCI_A_UART_EVEN_PARITY
//! \param msborLsbFirst controls direction of receive and transmit shift
//!        register.
//!        Valid values are:
//!        - \b USCI_A_UART_MSB_FIRST
//!        - \b USCI_A_UART_LSB_FIRST [Default]
//! \param numberofStopBits indicates one/two STOP bits
//!        Valid values are:
//!        - \b USCI_A_UART_ONE_STOP_BIT [Default]
//!        - \b USCI_A_UART_TWO_STOP_BITS
//! \param uartMode selects the mode of operation
//!        Valid values are:
//!        - \b USCI_A_UART_MODE [Default]
//!        - \b USCI_A_UART_IDLE_LINE_MULTI_PROCESSOR_MODE
//!        - \b USCI_A_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE
//!        - \b USCI_A_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE
//! \param overSampling indicates low frequency or oversampling baud generation
//!        Valid values are:
//!        - \b USCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION
//!        - \b USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION
//!
//! Modified bits are \b UCPEN, \b UCPAR, \b UCMSB, \b UC7BIT, \b UCSPB, \b
//! UCMODEx and \b UCSYNC of \b UCAxCTL0 register; bits \b UCSSELx and \b
//! UCSWRST of \b UCAxCTL1 register.
//!
//! \return STATUS_SUCCESS or STATUS_FAIL of the initialization process
//
//*****************************************************************************
void USCI_A_UART_initAdvance( usci_a_uart_t *in_ptr,
                              uint8_t selectClockSource,
                              uint16_t clockPrescalar,
                              uint8_t firstModReg,
                              uint8_t secondModReg,
                              uint8_t parity,
                              uint8_t msborLsbFirst,
                              uint8_t numberofStopBits,
                              uint8_t uartMode,
                              uint16_t overSampling
                              )
{

  
        //Disable the USCI Module
        HWREG8(in_ptr->UCAxCTL1) |= UCSWRST;

        //Clock source select
        HWREG8(in_ptr->UCAxCTL1) &= ~UCSSEL_3;
        HWREG8(in_ptr->UCAxCTL1) |= selectClockSource;

        //MSB, LSB select
        HWREG8(in_ptr->UCAxCTL0) &= ~UCMSB;
        HWREG8(in_ptr->UCAxCTL0) |= msborLsbFirst;


        //UCSPB = 0(1 stop bit) OR 1(2 stop bits)
        HWREG8(in_ptr->UCAxCTL0) &= ~UCSPB;
        HWREG8(in_ptr->UCAxCTL0) |= numberofStopBits;


        //Parity
        switch (parity) {
        case USCI_A_UART_NO_PARITY:
                //No Parity
                HWREG8(in_ptr->UCAxCTL0) &= ~UCPEN;
                break;
        case USCI_A_UART_ODD_PARITY:
                //Odd Parity
                HWREG8(in_ptr->UCAxCTL0) |= UCPEN;
                HWREG8(in_ptr->UCAxCTL0) &= ~UCPAR;
                break;
        case USCI_A_UART_EVEN_PARITY:
                //Even Parity
                HWREG8(in_ptr->UCAxCTL0) |= UCPEN;
                HWREG8(in_ptr->UCAxCTL0) |= UCPAR;
                break;
        }

        //Modulation Control Registers
        HWREG8(in_ptr->UCAxBR0 ) = clockPrescalar;
        HWREG8(in_ptr->UCAxMCTL) = ((firstModReg << 4) + (secondModReg << 1) +
                                              overSampling );

        //Asynchronous mode & 8 bit character select & clear mode
        HWREG8(in_ptr->UCAxCTL0) &=  ~(UCSYNC +
                                                 UC7BIT +
                                                 UCMODE_3
                                                 );

        //Configure  UART mode.
        HWREG8(in_ptr->UCAxCTL0) |= uartMode;

        //Reset UCRXIE, UCBRKIE, UCDORM, UCTXADDR, UCTXBRK
        HWREG8(in_ptr->UCAxCTL1)  &= ~(UCRXEIE + UCBRKIE + UCDORM +
                                                 UCTXADDR + UCTXBRK
                                                 );

  
}

//*****************************************************************************
//
//! \brief Transmits a byte from the UART Module.
//!
//! This function will place the supplied data into UART trasmit data register
//! to start transmission
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//! \param transmitData data to be transmitted from the UART module
//!
//! Modified bits of \b UCAxTXBUF register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_UART_transmitData( usci_a_uart_t *in_ptr,
                               uint8_t transmitData
                               )
{
        
		//If interrupts are not used, poll for flags
        if (!(HWREG8(in_ptr->UCAxIE) & UCTXIE))
                //Poll for transmit interrupt flag
                    while (!(HWREG8(in_ptr->UCAxIFG) & UCTXIFG)) ;
		
        HWREG8(in_ptr->UCAxTXBUF) = transmitData;
}

//*****************************************************************************
//
//! \brief Receives a byte that has been sent to the UART Module.
//!
//! This function reads a byte of data from the UART receive data Register.
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//!
//! Modified bits of \b UCAxRXBUF register.
//!
//! \return Returns the byte received from by the UART module, cast as an
//!         uint8_t.
//
//*****************************************************************************
uint8_t USCI_A_UART_receiveData(usci_a_uart_t *in_ptr)
{
        //If interrupts are not used, poll for flags
        if (!(HWREG8(in_ptr->UCAxIE) & UCRXIE))
                //Poll for receive interrupt flag
                while (!(HWREG8(in_ptr->UCAxIFG) & UCRXIFG));
 
		
		
        return HWREG8(in_ptr->UCAxRXBUF);
}

//*****************************************************************************
//
//! \brief Enables individual UART interrupt sources.
//!
//! Enables the indicated UART interrupt sources.  The interrupt flag is first
//! and then the corresponfing interrupt is enabled. Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor. Does not clear interrupt flags.
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//! \param mask is the bit mask of the interrupt sources to be enabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b USCI_A_UART_RECEIVE_INTERRUPT - Receive interrupt
//!        - \b USCI_A_UART_TRANSMIT_INTERRUPT - Transmit interrupt
//!        - \b USCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT - Receive
//!           erroneous-character interrupt enable
//!        - \b USCI_A_UART_BREAKCHAR_INTERRUPT - Receive break character
//!           interrupt enable
//!
//! Modified bits of \b UCAxCTL1 register and bits of \b UCAxIE register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_UART_enableInterrupt(usci_a_uart_t *in_ptr,
                                 uint8_t mask
                                 )
{

        uint8_t locMask;

         if (locMask = (mask & (USCI_A_UART_RECEIVE_INTERRUPT 
                               | USCI_A_UART_TRANSMIT_INTERRUPT))) {

                //Clear interrupt flag
                //HWREG8(in_ptr->UCAxIFG) &= ~(locMask);
                //Enable Interrupt
                HWREG8(in_ptr->UCAxIE) |= locMask;
        }

        if (locMask = (mask & (USCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT
                               | USCI_A_UART_BREAKCHAR_INTERRUPT))) {

                //Enable Interrupt
                HWREG8(in_ptr->UCAxCTL1) |= locMask;
        }
}

//*****************************************************************************
//
//! \brief Disables individual UART interrupt sources.
//!
//! Disables the indicated UART interrupt sources. Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor.
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//! \param mask is the bit mask of the interrupt sources to be disabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b USCI_A_UART_RECEIVE_INTERRUPT - Receive interrupt
//!        - \b USCI_A_UART_TRANSMIT_INTERRUPT - Transmit interrupt
//!        - \b USCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT - Receive
//!           erroneous-character interrupt enable
//!        - \b USCI_A_UART_BREAKCHAR_INTERRUPT - Receive break character
//!           interrupt enable
//!
//! Modified bits of \b UCAxCTL1 register and bits of \b UCAxIE register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_UART_disableInterrupt(usci_a_uart_t *in_ptr,
                                  uint8_t mask
                                  )
{

        uint8_t locMask;

        if (locMask = (mask & (USCI_A_UART_RECEIVE_INTERRUPT
                               | USCI_A_UART_TRANSMIT_INTERRUPT))) {

                //Disable Interrupt
                HWREG8(in_ptr->UCAxIE) &= ~locMask;
        }

        if (locMask = (mask & (USCI_A_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT
                               | USCI_A_UART_BREAKCHAR_INTERRUPT))) {

                //Disable Interrupt
                HWREG8(in_ptr->UCAxCTL1) &= ~locMask;
        }
}


//*****************************************************************************
//
//! \brief Gets the current UART interrupt status.
//!
//! This returns the interrupt status for the UART module based on which flag
//! is passed.
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//! \param mask is the masked interrupt flag status to be returned.
//!        Mask value is the logical OR of any of the following:
//!        - \b USCI_A_UART_RECEIVE_INTERRUPT_FLAG - Receive interrupt flag
//!        - \b USCI_A_UART_TRANSMIT_INTERRUPT_FLAG - Transmit interrupt flag
//!
//! Modified bits of \b UCAxIFG register.
//!
//! \return Logical OR of any of the following:
//!         - \b USCI_A_UART_RECEIVE_INTERRUPT_FLAG Receive interrupt flag
//!         - \b USCI_A_UART_TRANSMIT_INTERRUPT_FLAG Transmit interrupt flag
//!         \n indicating the status of the masked flags
//
//*****************************************************************************
uint8_t USCI_A_UART_getInterruptStatus(usci_a_uart_t *in_ptr,
                                       uint8_t mask)
{
        return HWREG8(in_ptr->UCAxIFG) & mask;
}

//*****************************************************************************
//
//! \brief Clears UART interrupt sources.
//!
//! The UART interrupt source is cleared, so that it no longer asserts. The
//! highest interrupt flag is automatically cleared when an interrupt vector
//! generator is used.
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//! \param mask is a bit mask of the interrupt sources to be cleared.
//!        Mask value is the logical OR of any of the following:
//!        - \b USCI_A_UART_RECEIVE_INTERRUPT_FLAG - Receive interrupt flag
//!        - \b USCI_A_UART_TRANSMIT_INTERRUPT_FLAG - Transmit interrupt flag
//!
//! Modified bits of \b UCAxIFG register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_UART_clearInterruptFlag(usci_a_uart_t *in_ptr, uint8_t mask)
{

        //Clear the UART interrupt source.
        HWREG8(in_ptr->UCAxIFG) &= ~(mask);
}

//*****************************************************************************
//
//! \brief Enables the UART block.
//!
//! This will enable operation of the UART block.
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//!
//! Modified bits are \b UCSWRST of \b UCAxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_UART_enable(usci_a_uart_t *in_ptr)
{
        //Reset the UCSWRST bit to enable the USCI Module
        HWREG8(in_ptr->UCAxCTL1) &= ~(UCSWRST);
}

//*****************************************************************************
//
//! \brief Disables the UART block.
//!
//! This will disable operation of the UART block.
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//!
//! Modified bits are \b UCSWRST of \b UCAxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_UART_disable(usci_a_uart_t *in_ptr)
{
        //Set the UCSWRST bit to disable the USCI Module
        HWREG8(in_ptr->UCAxCTL1) |= UCSWRST;
		
		USCI_A_UART_disableInterrupt(in_ptr, (USCI_A_UART_RECEIVE_INTERRUPT +USCI_A_UART_TRANSMIT_INTERRUPT));
}

/******************************************************************************
****                                                                       ****
**                                                                           **
usci_uart_rx_count()

Returns:  Number of characters currently in rx0 buffer.      

**                                                                           **
****                                                                       ****
******************************************************************************/
/*uint16_t USCI_A_UART_rxbuff_count(usci_a_uart_t *in_ptr) {
  return rbuff_count(&in_ptr->RxBuff);
}*/ /* usci_uart_rx_count() */

/******************************************************************************
****                                                                       ****
**                                                                           **
usci_uart_tx_free()

Returns:  Maximimum number of characters that can currently
              be placed into the tx0 buffer.      

**                                                                           **
****                                                                       ****
******************************************************************************/
/*uint16_t USCI_A_UART_txbuff_free(usci_a_uart_t *in_ptr) {
  return rbuff_free(&in_ptr->txBuff);
}*/ /* usci_uart_tx_free() */

/******************************************************************************
****                                                                       ****
**                                                                           **
usci_uart_getchar()

Removes a character (if present) from the hardware UART's
receiver ring buffer returns it as an int.

Algorithm:  if (char in Rx ring buffer)
              get character from Rx ring buffer
              wrap Rx ring buffer's output ptr if req'd
              decrement Rx ring buffer count
              return character
            else
              return 0  

Notes:     If/when changes to rx0count are not atomic,
             protection is required. As long as it's a
             (16-bit) int, no protection is required.

Returns:    character if a character was present.
            0 if buffer was empty.

**                                                                           **
****                                                                       ****
******************************************************************************/
/*uint8_t  USCI_A_UART_getchar(usci_a_uart_t *in_ptr) {
  return rbuffer_getchar(&in_ptr->rxBuff);
}*/ /* usci_uart_getchar() */

/******************************************************************************
****                                                                       ****
**                                                                           **
usci_uart_putchar()

Puts the character into UART0's transmitter's ring buffer
if room is available.

We enable Tx0 interrupts unconditionally because whenever
we put a character into the ring buffer, we want the ISR
to occur. Since the interrupt flag is initially set, enabling
interrupts causes us to vector to the ISR, and send the char.

Algorithm:  if (room in Tx ring buffer)
              put char in Tx ring buffer
              wrap Tx ring buffer's input ptr if req'd
              update Tx buffer count
              enable Tx interrupts
              return c
            else
              return EOF

Notes:     If/when changes to tx0count are not atomic,
             protection is required. As long as it's a
             (16-bit) int, no protection is required.

Returns:    0 if all chars placed.
            num chars not placed if buffer was full.

**                                                                           **
****                                                                       ****
******************************************************************************/
/*uint16_t  USCI_A_UART_putchars(usci_a_uart_t *in_ptr, uint8_t *in, uint16_t inLength) {
  return rbuffer_putchar(&in_ptr->txBuff, in, inLength);
}*/ /* usci_uart_putchar() */


/******************************************************************************
****                                                                       ****
**                                                                           **
usci_uart_puts()

Writes the string of characters passed to it out Tx0.

We enable Tx0 interrupts unconditionally because whenever
we put a character into the ring buffer, we want the ISR
to occur. Since the interrupt flag is initially set, enabling
interrupts causes us to vector to the ISR, and send the char.

Algorithm:  while (end-of-string not reached)
              put char in Tx ring buffer
              increment local pointer
        
Notes:      Differs from stdio.h's puts():
              const char * argument (not char *)
              always returns 0
              assumes stdout is Tx0

Returns:    number of chars placed in buffer
            NOTE: May not place all of chars in string!
                  It maxes out at the buffer's max length

**                                                                           **
****                                                                       ****
******************************************************************************/
/*uint16_t USCI_A_UART_puts(usci_a_uart_t *in_ptr, const uint8_t *c) {
  uint8_t *start = c;

  while (*c) {
    if (rbuffer_putchar(&in_ptr->txBuff, *c) == EOF_RETURN_CODE) {
      break;
    }
  }
  return c - start;
}*/ /* usci_uart_puts() */

/******************************************************************************
****                                                                       ****
**                                                                           **
usci_uart_outchar()

Takes a character from  UART0's transmitter's ring buffer
and sends it out via UART0's transmitter..

To be called from within UART0's transmitter's interrupt
service routine (i.e. the ISR at UART0TX_VECTOR).

Algorithm:  take char from Tx ring buffer and place into
             UART's TXBUF
            wrap Tx ring buffer's output pointer if required
            update Tx buffer count
            if (Tx buffer empty)
             disable transmitter interrupts

Notes:    One could argue that one should check if for
           non-zero <tx1Count> before proceeding ... however,
          the only reason we should arrive at the
          UART1TX_VECTOR ISR is because the transmitter
           is ready for another character, AND we just
          enabled its interrupts after adding a character
          to the ring buffer, so a check should never be
          necessary.


Returns:    --

**                                                                           **
****                                                                       ****
******************************************************************************/
/*uint8_t USCI_A_UART_outchar(usci_a_uart_t *in_ptr) {
  uint8_t byte;
  byte = rbuffer_getchar(&in_ptr->rxBuff);

  if (byte != EOF_RETURN_CODE) {
    USCI_A_UART_transmitData(in_ptr, byte);
  }
  return byte;
}*/ /* usci_uart_outchar() */

/******************************************************************************
****                                                                       ****
**                                                                           **
usci_uart_inchar()

Takes a character from  UART0's receiver
and puts it into UART0's receiver ring buffer

To be called from within UART0's receiver's interrupt
service routine (i.e. the ISR at UART0RX_VECTOR).

Algorithm:  if (room in Rx ring buffer)
             take char from UART rcvr and place in Rx
             ring buffer
            wrap Rx ring buffer's input pointer if req'd
            increment Rx ring buffer count
			
			if size of ring buffer equals Device_RXmsg size
			then return 1

Notes:      No error checking on incoming chars.

Returns:   --

**                                                                           **
****                                                                       ****
******************************************************************************/
/*void USCI_A_UART_inchar(usci_a_uart_t *in_ptr) {
  if (rbuffer_free(&in_ptr->rxBuff) > 0) {
    uint8_t byte = USCI_A_UART_receiveData(in_ptr);
    rbuffer_putchar(&in_ptr->rxBuff, byte);
  }
}*/ /* usci_uart_inchar() */

//*****************************************************************************
//
//! \brief Gets the current UART status flags.
//!
//! This returns the status for the UART module based on which flag is passed.
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//! \param mask is the masked interrupt flag status to be returned.
//!        Mask value is the logical OR of any of the following:
//!        - \b USCI_A_UART_LISTEN_ENABLE
//!        - \b USCI_A_UART_FRAMING_ERROR
//!        - \b USCI_A_UART_OVERRUN_ERROR
//!        - \b USCI_A_UART_PARITY_ERROR
//!        - \b USCI_A_UART_BREAK_DETECT
//!        - \b USCI_A_UART_RECEIVE_ERROR
//!        - \b USCI_A_UART_ADDRESS_RECEIVED
//!        - \b USCI_A_UART_IDLELINE
//!        - \b USCI_A_UART_BUSY
//!
//! Modified bits of \b UCAxSTAT register.
//!
//! \return Logical OR of any of the following:
//!         - \b USCI_A_UART_LISTEN_ENABLE
//!         - \b USCI_A_UART_FRAMING_ERROR
//!         - \b USCI_A_UART_OVERRUN_ERROR
//!         - \b USCI_A_UART_PARITY_ERROR
//!         - \b USCI_A_UART_BREAK_DETECT
//!         - \b USCI_A_UART_RECEIVE_ERROR
//!         - \b USCI_A_UART_ADDRESS_RECEIVED
//!         - \b USCI_A_UART_IDLELINE
//!         - \b USCI_A_UART_BUSY
//!         \n indicating the status of the masked interrupt flags
//
//*****************************************************************************
uint8_t USCI_A_UART_queryStatusFlags(usci_a_uart_t *in_ptr,
                                     uint8_t mask)
{
        return HWREG8(in_ptr->UCAxSTAT) & mask;
}

//*****************************************************************************
//
//! \brief Sets the UART module in dormant mode
//!
//! Puts USCI in sleep mode. Only characters that are preceded by an idle-line
//! or with address bit set UCRXIFG. In UART mode with automatic baud-rate
//! detection, only the combination of a break and synch field sets UCRXIFG.
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//!
//! Modified bits of \b UCAxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_UART_setDormant(usci_a_uart_t *in_ptr)
{
        HWREG8(in_ptr->UCAxCTL1) |= UCDORM;
}

//*****************************************************************************
//
//! \brief Re-enables UART module from dormant mode
//!
//! Not dormant. All received characters set UCRXIFG.
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//!
//! Modified bits are \b UCDORM of \b UCAxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_UART_resetDormant(usci_a_uart_t *in_ptr)
{
        HWREG8(in_ptr->UCAxCTL1) &= ~UCDORM;
}

//*****************************************************************************
//
//! \brief Transmits the next byte to be transmitted marked as address
//! depending on selected multiprocessor mode
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//! \param transmitAddress is the next byte to be transmitted
//!
//! Modified bits of \b UCAxTXBUF register and bits of \b UCAxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_UART_transmitAddress(usci_a_uart_t *in_ptr,
                                 uint8_t transmitAddress)
{
        //Set UCTXADDR bit
        HWREG8(in_ptr->UCAxCTL1) |= UCTXADDR;

        //Place next byte to be sent into the transmit buffer
        HWREG8(in_ptr->UCAxTXBUF) = transmitAddress;
}

//*****************************************************************************
//
//! \brief Transmit break.
//!
//! Transmits a break with the next write to the transmit buffer. In UART mode
//! with automatic baud-rate detection,
//! USCI_A_UART_AUTOMATICBAUDRATE_SYNC(0x55) must be written into UCAxTXBUF to
//! generate the required break/synch fields. Otherwise, DEFAULT_SYNC(0x00)
//! must be written into the transmit buffer. Also ensures module is ready for
//! transmitting the next data.
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//!
//! Modified bits of \b UCAxTXBUF register and bits of \b UCAxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_UART_transmitBreak(usci_a_uart_t *in_ptr)
{
        //Set UCTXADDR bit
        HWREG8(in_ptr->UCAxCTL1) |= UCTXBRK;

        //If current mode is automatic baud-rate detection
        if (USCI_A_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE ==
            (HWREG8(in_ptr->UCAxCTL0) &
             USCI_A_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE))
                HWREG8(in_ptr->UCAxTXBUF) = USCI_A_UART_AUTOMATICBAUDRATE_SYNC;
        else
                HWREG8(in_ptr->UCAxTXBUF) = DEFAULT_SYNC;

        //If interrupts are not used, poll for flags
        if (!(HWREG8(in_ptr->UCAxIE) & UCTXIE))
                //Poll for transmit interrupt flag
                while (!(HWREG8(in_ptr->UCAxIFG) & UCTXIFG)) ;
}

//*****************************************************************************
//
//! \brief Returns the address of the RX Buffer of the UART for the DMA module.
//!
//! Returns the address of the UART RX Buffer. This can be used in conjunction
//! with the DMA to store the received data directly to memory.
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//!
//! \return Address of RX Buffer
//
//*****************************************************************************
uint32_t USCI_A_UART_getReceiveBufferAddressForDMA(usci_a_uart_t *in_ptr)
{
        return in_ptr->UCAxRXBUF;
}

//*****************************************************************************
//
//! \brief Returns the address of the TX Buffer of the UART for the DMA module.
//!
//! Returns the address of the UART TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \param baseAddress is the base address of the USCI_A_UART module.
//!
//! \return Address of TX Buffer
//
//*****************************************************************************
uint32_t USCI_A_UART_getTransmitBufferAddressForDMA(usci_a_uart_t *in_ptr)
{
        return in_ptr->UCAxTXBUF;
}

//*****************************************************************************
//
//! Close the doxygen group for usci_a_uart_api
//! @}
//
//*****************************************************************************

#endif
