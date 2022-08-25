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
// usci_a_spi.c - Driver for the usci_a_spi Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup usci_a_spi_api
//! @{
//
//*****************************************************************************

#include "inc/hw_regaccess.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/booltype.h"

#ifdef __MSP430_HAS_USCI_Ax__
#include "usci_a_spi.h"

void USCI_A_SPI_initregs(usci_a_spi_t *in_ptr)
{

	in_ptr->UCAxIE		=	in_ptr->baseAddress + OFS_UCAxIE;
	in_ptr->UCAxIFG		=	in_ptr->baseAddress + OFS_UCAxIFG;
	in_ptr->UCAxCTL1	=	in_ptr->baseAddress + OFS_UCAxCTL1;
	in_ptr->UCAxCTL0	=	in_ptr->baseAddress + OFS_UCAxCTL0;
	in_ptr->UCAxBRW		=	in_ptr->baseAddress + OFS_UCAxBR0;
	in_ptr->UCAxBR1		= 	in_ptr->baseAddress + OFS_UCAxBR1;
	in_ptr->UCAxMCTL	=	in_ptr->baseAddress + OFS_UCAxMCTL;
	in_ptr->UCAxSTAT	=	in_ptr->baseAddress + OFS_UCAxSTAT;
	in_ptr->UCAxTXBUF	=	in_ptr->baseAddress + OFS_UCAxTXBUF;
	in_ptr->UCAxRXBUF	=	in_ptr->baseAddress + OFS_UCAxRXBUF;

}
//*****************************************************************************
//
//! \brief Initializes the SPI Master block.
//!
//! Upon successful initialization of the SPI master block, this function will
//! have set the bus speed for the master, but the SPI Master block still
//! remains disabled and must be enabled with USCI_A_SPI_enable()
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param selectClockSource selects Clock source.
//!        Valid values are:
//!        - \b USCI_A_SPI_CLOCKSOURCE_ACLK
//!        - \b USCI_A_SPI_CLOCKSOURCE_SMCLK
//! \param clockSourceFrequency is the frequency of the slected clock source
//! \param desiredSpiClock is the desired clock rate for SPI communication
//! \param msbFirst controls the direction of the receive and transmit shift
//!        register.
//!        Valid values are:
//!        - \b USCI_A_SPI_MSB_FIRST
//!        - \b USCI_A_SPI_LSB_FIRST [Default]
//! \param clockPhase is clock phase select.
//!        Valid values are:
//!        - \b USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
//!           [Default]
//!        - \b USCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
//! \param clockPolarity
//!        Valid values are:
//!        - \b USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
//!        - \b USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW [Default]
//!
//! Modified bits are \b UCCKPH, \b UCCKPL, \b UC7BIT and \b UCMSB of \b
//! UCAxCTL0 register; bits \b UCSSELx and \b UCSWRST of \b UCAxCTL1 register.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
bool_t USCI_A_SPI_masterInit(usci_a_spi_t *in_ptr,
uint32_t baseAddressin,
                           uint8_t selectClockSource,
                           uint32_t clockSourceFrequency,
                           uint32_t desiredSpiClock,
                           uint8_t msbFirst,
                           uint8_t clockPhase,
                           uint8_t clockPolarity
                           )
{
	in_ptr->baseAddress = baseAddressin;
        USCI_A_SPI_initregs(in_ptr);
        //Disable the USCI Module
        HWREG8(in_ptr->UCAxCTL1) |= UCSWRST;

        //Reset OFS_UCAxCTL0 values
        HWREG8(in_ptr->UCAxCTL0) &= ~(UCCKPH + UCCKPL + UC7BIT + UCMSB + UCMST + UCMODE_3 + UCSYNC);


        //Reset OFS_UCAxCTL1 values
        HWREG8(in_ptr->UCAxCTL1) &= ~(UCSSEL_3);

        //Select Clock
        HWREG8(in_ptr->UCAxCTL1) |= selectClockSource;


        HWREG8(in_ptr->UCAxBRW) =
                (uint16_t)(clockSourceFrequency / desiredSpiClock);


        /*
         * Configure as SPI master mode.
         * Clock phase select, polarity, msb
         * UCMST = Master mode
         * UCSYNC = Synchronous mode
         * UCMODE_0 = 3-pin SPI
         */
        HWREG8(in_ptr->UCAxCTL0) |= (
                msbFirst +
                clockPhase +
                clockPolarity +
                UCMST +
                UCSYNC +
                UCMODE_0
                );
        //No modulation
        HWREG8(in_ptr->UCAxMCTL) = 0;

        return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! \brief Initializes the SPI Master clock.At the end of this function call,
//! SPI module is left enabled.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param clockSourceFrequency is the frequency of the slected clock source
//! \param desiredSpiClock is the desired clock rate for SPI communication
//!
//! Modified bits of \b UCAxBRW register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_SPI_masterChangeClock(usci_a_spi_t *in_ptr,
                                  uint32_t clockSourceFrequency,
                                  uint32_t desiredSpiClock
                                  )
{
        //Disable the USCI Module
        HWREG8(in_ptr->UCAxCTL1) |= UCSWRST;

        HWREG8(in_ptr->UCAxBRW) =
                (uint16_t)(clockSourceFrequency / desiredSpiClock);

        //Reset the UCSWRST bit to enable the USCI Module
        HWREG8(in_ptr->UCAxCTL1) &= ~(UCSWRST);
}

//*****************************************************************************
//
//! \brief Initializes the SPI Slave block.
//!
//! Upon successful initialization of the SPI slave block, this function will
//! have initialized the slave block, but the SPI Slave block still remains
//! disabled and must be enabled with USCI_A_SPI_enable()
//!
//! \param baseAddress is the base address of the SPI Slave module.
//! \param msbFirst controls the direction of the receive and transmit shift
//!        register.
//!        Valid values are:
//!        - \b USCI_A_SPI_MSB_FIRST
//!        - \b USCI_A_SPI_LSB_FIRST [Default]
//! \param clockPhase is clock phase select.
//!        Valid values are:
//!        - \b USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
//!           [Default]
//!        - \b USCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
//! \param clockPolarity
//!        Valid values are:
//!        - \b USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
//!        - \b USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW [Default]
//!
//! Modified bits are \b UCMSB, \b UCMST, \b UC7BIT, \b UCCKPL, \b UCCKPH and
//! \b UCMODE of \b UCAxCTL0 register; bits \b UCSWRST of \b UCAxCTL1 register.
//!
//! \return STATUS_SUCCESS
//
//*****************************************************************************
bool_t USCI_A_SPI_slaveInit(usci_a_spi_t *in_ptr,
uint32_t baseAddressin,
                          uint8_t msbFirst,
                          uint8_t clockPhase,
                          uint8_t clockPolarity
                          )
{
        in_ptr->baseAddress = baseAddressin;
        USCI_A_SPI_initregs(in_ptr);

        //Disable USCI Module
        HWREG8(in_ptr->UCAxCTL1)  |= UCSWRST;

        //Reset OFS_UCAxCTL0 register
        HWREG8(in_ptr->UCAxCTL0) &= ~(UCMSB +
                                                UC7BIT +
                                                UCMST +
                                                UCCKPL +
                                                UCCKPH +
                                                UCMODE_3
                                                );


        //Clock polarity, phase select, msbFirst, SYNC, Mode0
        HWREG8(in_ptr->UCAxCTL0) |= ( clockPhase +
                                                clockPolarity +
                                                msbFirst +
                                                UCSYNC +
                                                UCMODE_0
                                                );


        return STATUS_SUCCESS;
}

//*****************************************************************************
//
//! \brief Changes the SPI colock phase and polarity.At the end of this
//! function call, SPI module is left enabled.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param clockPhase is clock phase select.
//!        Valid values are:
//!        - \b USCI_A_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT
//!           [Default]
//!        - \b USCI_A_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT
//! \param clockPolarity
//!        Valid values are:
//!        - \b USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_HIGH
//!        - \b USCI_A_SPI_CLOCKPOLARITY_INACTIVITY_LOW [Default]
//!
//! Modified bits are \b UCCKPL and \b UCCKPH of \b UCAxCTL0 register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_SPI_changeClockPhasePolarity(usci_a_spi_t *in_ptr,
                                         uint8_t clockPhase,
                                         uint8_t clockPolarity
                                         )
{

        //Disable the USCI Module
        HWREG8(in_ptr->UCAxCTL1) |= UCSWRST;

        HWREG8(in_ptr->UCAxCTL0) &= ~(UCCKPH + UCCKPL);

        HWREG8(in_ptr->UCAxCTL0) |= (
                clockPhase +
                clockPolarity
                );

        //Reset the UCSWRST bit to enable the USCI Module
        HWREG8(in_ptr->UCAxCTL1) &= ~(UCSWRST);
}

//*****************************************************************************
//
//! \brief Transmits a byte from the SPI Module.
//!
//! This function will place the supplied data into SPI trasmit data register
//! to start transmission
//!
//! \param baseAddress is the base address of the SPI module.
//! \param transmitData data to be transmitted from the SPI module
//!
//! \return None
//
//*****************************************************************************
void USCI_A_SPI_transmitData( usci_a_spi_t *in_ptr,
                              uint8_t transmitData
                              )
{
        HWREG8(in_ptr->UCAxTXBUF) = transmitData;
}

//*****************************************************************************
//
//! \brief Receives a byte that has been sent to the SPI Module.
//!
//! This function reads a byte of data from the SPI receive data Register.
//!
//! \param baseAddress is the base address of the SPI module.
//!
//! \return Returns the byte received from by the SPI module, cast as an
//!         uint8_t.
//
//*****************************************************************************
uint8_t USCI_A_SPI_receiveData(usci_a_spi_t *in_ptr)
{
        return HWREG8(in_ptr->UCAxRXBUF);
}

//*****************************************************************************
//
//! \brief Enables individual SPI interrupt sources.
//!
//! Enables the indicated SPI interrupt sources.  Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor. <b>Does not clear interrupt flags.
//!
//! \param baseAddress is the base address of the SPI module.
//! \param mask is the bit mask of the interrupt sources to be enabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b USCI_A_SPI_TRANSMIT_INTERRUPT
//!        - \b USCI_A_SPI_RECEIVE_INTERRUPT
//!
//! Modified bits of \b UCAxIE register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_SPI_enableInterrupt(usci_a_spi_t *in_ptr,
                                uint8_t mask
                                )
{


        HWREG8(in_ptr->UCAxIE) |= mask;
}

//*****************************************************************************
//
//! \brief Disables individual SPI interrupt sources.
//!
//! Disables the indicated SPI interrupt sources. Only the sources that are
//! enabled can be reflected to the processor interrupt; disabled sources have
//! no effect on the processor.
//!
//! \param baseAddress is the base address of the SPI module.
//! \param mask is the bit mask of the interrupt sources to be disabled.
//!        Mask value is the logical OR of any of the following:
//!        - \b USCI_A_SPI_TRANSMIT_INTERRUPT
//!        - \b USCI_A_SPI_RECEIVE_INTERRUPT
//!
//! Modified bits of \b UCAxIE register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_SPI_disableInterrupt(usci_a_spi_t *in_ptr,
                                 uint8_t mask
                                 )
{


        HWREG8(in_ptr->UCAxIE) &= ~mask;
}

//*****************************************************************************
//
//! \brief Gets the current SPI interrupt status.
//!
//! This returns the interrupt status for the SPI module based on which flag is
//! passed.
//!
//! \param baseAddress is the base address of the SPI module.
//! \param mask is the masked interrupt flag status to be returned.
//!        Mask value is the logical OR of any of the following:
//!        - \b USCI_A_SPI_TRANSMIT_INTERRUPT
//!        - \b USCI_A_SPI_RECEIVE_INTERRUPT
//!
//! \return The current interrupt status as the mask of the set flags
//!         Return Logical OR of any of the following:
//!         - \b USCI_A_SPI_TRANSMIT_INTERRUPT
//!         - \b USCI_A_SPI_RECEIVE_INTERRUPT
//!         \n indicating the status of the masked interrupts
//
//*****************************************************************************
uint8_t USCI_A_SPI_getInterruptStatus(usci_a_spi_t *in_ptr,
                                      uint8_t mask
                                      )
{


        return HWREG8(in_ptr->UCAxIFG) & mask;
}

//*****************************************************************************
//
//! \brief Clears the selected SPI interrupt status flag.
//!
//! \param baseAddress is the base address of the SPI module.
//! \param mask is the masked interrupt flag to be cleared.
//!        Mask value is the logical OR of any of the following:
//!        - \b USCI_A_SPI_TRANSMIT_INTERRUPT
//!        - \b USCI_A_SPI_RECEIVE_INTERRUPT
//!
//! Modified bits of \b UCAxIFG register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_SPI_clearInterruptFlag(usci_a_spi_t *in_ptr,
                                   uint8_t mask
                                   )
{

        HWREG8(in_ptr->UCAxIFG) &=  ~mask;
}

//*****************************************************************************
//
//! \brief Enables the SPI block.
//!
//! This will enable operation of the SPI block.
//!
//! \param baseAddress is the base address of the USCI SPI module.
//!
//! Modified bits are \b UCSWRST of \b UCAxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_SPI_enable(usci_a_spi_t *in_ptr)
{
        //Reset the UCSWRST bit to enable the USCI Module
        HWREG8(in_ptr->UCAxCTL1) &= ~(UCSWRST);
}

//*****************************************************************************
//
//! \brief Disables the SPI block.
//!
//! This will disable operation of the SPI block.
//!
//! \param baseAddress is the base address of the USCI SPI module.
//!
//! Modified bits are \b UCSWRST of \b UCAxCTL1 register.
//!
//! \return None
//
//*****************************************************************************
void USCI_A_SPI_disable(usci_a_spi_t *in_ptr)
{
        //Set the UCSWRST bit to disable the USCI Module
        HWREG8(in_ptr->UCAxCTL1) |= UCSWRST;
}

//*****************************************************************************
//
//! \brief Returns the address of the RX Buffer of the SPI for the DMA module.
//!
//! Returns the address of the SPI RX Buffer. This can be used in conjunction
//! with the DMA to store the received data directly to memory.
//!
//! \param baseAddress is the base address of the SPI module.
//!
//! \return the address of the RX Buffer
//
//*****************************************************************************
uint32_t USCI_A_SPI_getReceiveBufferAddressForDMA(usci_a_spi_t *in_ptr)
{
        return in_ptr->UCAxRXBUF;
}

//*****************************************************************************
//
//! \brief Returns the address of the TX Buffer of the SPI for the DMA module.
//!
//! Returns the address of the SPI TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \param baseAddress is the base address of the SPI module.
//!
//! \return the address of the TX Buffer
//
//*****************************************************************************
uint32_t USCI_A_SPI_getTransmitBufferAddressForDMA(usci_a_spi_t *in_ptr)
{
        return in_ptr->UCAxTXBUF;
}

//*****************************************************************************
//
//! \brief Indicates whether or not the SPI bus is busy.
//!
//! This function returns an indication of whether or not the SPI bus is
//! busy.This function checks the status of the bus via UCBBUSY bit
//!
//! \param baseAddress is the base address of the SPI module.
//!
//! \return USCI_A_SPI_BUSY if the SPI module trasmitting or receiving is busy;
//!         otherwise, returns USCI_A_SPI_NOT_BUSY.
//!         Return one of the following:
//!         - \b USCI_A_SPI_BUSY
//!         - \b USCI_A_SPI_NOT_BUSY
//!         \n indicating if the USCI_A_SPI is busy
//
//*****************************************************************************
uint8_t USCI_A_SPI_isBusy(usci_a_spi_t *in_ptr)
{
        //Return the bus busy status.
        return HWREG8(in_ptr->UCAxSTAT) & UCBUSY;
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for usci_a_spi_api
//! @}
//
//*****************************************************************************
