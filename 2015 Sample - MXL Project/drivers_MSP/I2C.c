/************************************************************************************
 FileName:     I2C.c
 Dependencies: See INCLUDES section
 Processor:    MSP430
 Hardware:       
 Complier:     CrossStudio for MSP430
 Company:      University of Michigan RAX Lab
*************************************************************************************
 File Description:
              The firmware contained within, Initializes/Reads/Writes/ the MSP430 
              I2C function on UART0.  
              
 Change History:
  Rev   Date         Author            Description   
  0.5   ??/??/????   katysat Team      Prototyped I2C code, not functioning properly
  1.0   07/07/2009   Jacob Beningo     Initial release
  1.1   09/02/2009   Jacob Beningo     Update to Initialization routine to initialize
                                       properly if SPI or UART were used prior to I2C
  1.2   10/27/2009   Jacob Beningo     Updated I2C initialization to handle varying
                                       baud rates and to check input variables.
                                       Adjusted error sizes to be uint16_t.
  2.0   06/30/2014   Joshua McCready   Re-wrote i2c functionality for msp430f2618
                                       using the original RAX code as a starting point. 
  2.1   10/17/2014   James Cutler      Memory leak found in i2c_read functionality, bu JWC
                     Joshua McCready   additional update for I2C_NOT_BUSY function 
    

License:
       Copyright (C)  2009 Radio Aurora Explorer Team, University of
       Michigan and Jacob Beningo
    
       This program is free software: you can redistribute it and/or modify
       it under the terms of the GNU General Public License as published by
       the Free Software Foundation, either version 3 of the License, or
       (at your option) any later version.
    
       This program is distributed in the hope that it will be useful,
       but WITHOUT ANY WARRANTY; without even the implied warranty of
       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
       GNU General Public License for more details.
    
       You should have received a copy of the GNU General Public License
       along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
       The GNU GPL license has been modified with the following clause:
    
       "If publishable results are created from the Program or a derivative
        of the Program, the original authors must be acknowledged in any
        published, presented or distributed work. An appropriate
        acknowledgment for a conference paper might include:
    
        Results obtained through the use of software created in part at the
        University of Michigan by the Radio Aurora Explorer Team (RAX Team),
        James Cutler (jwcutler@umich.edu) and
        Jacob Beningo (jacob.beningo@gmail.com), 2009."
        
        You can contact the authors through Prof James Cutler,
        jwcutler@umich.edu, 1320 Beal Ave, Ann Arbor, MI 48105.   

Notes:
        None
************************************************************************************/

/** INCLUDES ***********************************************************************/
#include <msp430.h>
#include "I2C.h"
#include "rbuffer.h"

#include <__cross_studio_io.h>
/** V A R I A B L E S ***************************************************************/
rbuffer_t I2C_RX_buf;
rbuffer_t I2C_TX_buf; 
uint8_t RXCounter;
uint8_t SINGLE_BYTE_RX; 
uint8_t ERROR_I2C_ISR_CONFLICT = 0;

/*************************************************************************************
 * Function:        void I2C_Init_Master(void)
 *
 * PreCondition:    None
 *
 *  \param selectClockSource is the clocksource.
//!        Valid values are:
//!        - \b USCI_B_I2C_CLOCKSOURCE_ACLK
//!        - \b USCI_B_I2C_CLOCKSOURCE_SMCLK
//! \param i2cClk is the rate of the clock supplied to the I2C module.
//! \param dataRate set up for selecting data transfer rate.
//!        Valid values are:
//!        - \b USCI_B_I2C_SET_DATA_RATE_400KBPS
//!        - \b USCI_B_I2C_SET_DATA_RATE_100KBPS
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Initializes the I2C bus based on the given inputs
 *
 * Note:            1) Currently hardcoded for specific pins
 *                  2) I2C baud rate calculated using:
 *                     (I2CPSC +1) x (I2CSCLH + 2) (I2CPSC + 1) x (I2CSCLL + 2)
 *                  3) Values of clk_div greater than 4 are set to 4 to prevent 
 *                     unpredictable behavior.
 *************************************************************************************/
void I2C_Init_Master(uint8_t selectClockSource,
                     uint32_t i2cClk,
                     uint32_t dataRate)
{
 uint16_t preScalarValue;
/********************************************************************************
 * Initialize the SCK and SDA lines on Port 3
 *******************************************************************************/
  P3SEL |= BIT0+BIT1;                               // Set P3.1,3.0 as I2C                  

/********************************************************************************
 * Set UCSWRST to avoid unpredictable behaviour
 *******************************************************************************/
  UCB0CTL1 = UCSWRST;	
/********************************************************************************
 * Setup procedure for master mode 
 *******************************************************************************/
  WDTCTL = WDTPW + WDTHOLD;                           // Stop WDT
  UCB0CTL0 |= UCMODE_3+UCSYNC+UCMST;                  // Set mode to Master I2C, synchronous mode
  UCB0I2COA |= OWN_ADDR; 			      // Set Own address as defined in i2c.h  	
  UCB0CTL0 &= ~(UCSLA10);			      // Set slave to 7 bit address     	   
/********************************************************************************
 * Select Clock source and implement clock division for data rate 
 *******************************************************************************/
  //Configure I2C clock source
  UCB0CTL1 = (selectClockSource + UCSWRST );

  /*
   * Compute the clock divider that achieves the fastest speed less than or
   * equal to the desired speed.  The numerator is biased to favor a larger
   * clock divider so that the resulting clock is always less than or equal
   * to the desired clock, never greater.
   */
  preScalarValue = (unsigned short)(i2cClk / dataRate);
  UCB0BR0 = preScalarValue;
  UCB0CTL1 &= ~(UCSWRST);                             // Reset to continue I2C operation   	 

/********************************************************************************
 * Enable RX and TX interrupts 
 *******************************************************************************/
  IE2 |= UCB0TXIE+UCB0RXIE;                           // Enable Master interrupt for RX and TX
}


/*************************************************************************************
 * Function:        I2C_Write(uint8_t* command, uint8_t length, uint8_t slave_address)
 *
 * PreCondition:    I2C_Init_Master() ran
 *
 * Input:           uint8_t* command      - pointer to data that will be transmitted
 *                  uint8_t length        - length of the data to be transmitted
 *                  uint8_t slave_address - slave device address
 *
 * Output:          Sends the command to the specified slave device
 *
 * Side Effects:    None
 *
 * Overview:        Sends the command of the specified length, to the specified slave
 *                  device over I2C.
 *
 * Note:            None
*************************************************************************************/
uint16_t I2C_Write(uint8_t* command, uint8_t length, uint8_t slave_address)
{
  uint8_t tx_index = 0; 
/********************************************************************************
 * Wait for UART to not be busy.  If it takes too long return an error 
 *******************************************************************************/
 if(Wait_for_I2C_Not_Busy())
 {
   return ERROR_I2C_BUSY;
 }

/********************************************************************************
 * Set up registers to write data to the bus, copy command
 *******************************************************************************/
  UCB0I2CSA = slave_address;                        // Set the slave address          
  rbuffer_putchars(&I2C_TX_buf, command, (uint16_t) length);  // Load Output buffer
  UCB0CTL1 |= UCTR + UCTXSTT;                       // I2C TX, start condition
  __bis_SR_register(CPUOFF + GIE);                  // Enter LPM0 w/ interrupts
                                                    // Remain in LPM0 until all data
                                                    // is TX'd
  
/********************************************************************************
 * Wait for UART to not be busy.  If it takes too long return an error 
 *******************************************************************************/
  if(Wait_for_I2C_Not_Busy())
  {
   return ERROR_I2C_BUSY;
  }

 // TO DO: CHECK TO SEE IF NACK AND ARB INTERRUPTS NEED TO BE ENABLED
 // TO DO: FIX THIS ITS EXITING TOO EARLY 
/********************************************************************************
 * Was transmission successful?  Check for Lost Arbitration 
 *******************************************************************************/
 if((UCB0STAT & UCALIFG) != 0x01)
 {
  //If the I2C flag is not set then the communication was successful
  return I2C_NO_ERROR;
 }
 else
 {
  //Reset the Flag
  UCB0STAT &= ~UCALIFG;

  //If the I2C flag was set then the communication was not successful 
  return ERROR_I2C_LOSTARB_RECEIVED;
 }
 return I2C_NO_ERROR;
}


/*************************************************************************************
 * Function:        uint8_t Wait_for_I2C_Not_Busy(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Waits for the I2C USART to be ready
 *
 * Note:            This function should not be used on its own but as part of a write
 *                  or read operation on the I2C bus
*************************************************************************************/

uint8_t Wait_for_I2C_Not_Busy()
{
  uint16_t I2C_counter = 0;             /* I2C busy counter                    */

  //Wait for I2C to not be busy.  If it takes too long return an error 
  // JGM: Cutler and I both have had problems with this I changed the condition to 
  // (I2C_counter < (0XFFFF-1)) from (*I2C_counter < 0xFFFF) because it got stuck in
  // an infinite loop, but it doesnt make sense because I2C_counter should count up 
  // to 0xFFFF, so the condition should evaluate false... 
  while(I2C_counter < (0XFFFF-1))
  {
    ++I2C_counter;
   if (!(UCB0STAT & UCBBUSY))
    {
      return I2C_NO_ERROR;  
    }
  }
  
  // Throw error once the counter overflows. 
  return ERROR_I2C_BUSY;
}

/*************************************************************************************
 * Function:        uint8_t I2C_Read( uint16_t length, uint8_t slave_address)
 *
 * PreCondition:    I2C_Init_Master() and I2C_Write called
 *
 * Input:           uint16_t length       - length of the data to be read
 *                  uint8_t slave_address - address of the slave to be read
 *
 * Output:          See Side effects
 *
 * Side Effects:    Appends received data to the global ring buffer I2C_RX_buf. 
 *                  Received data must removed before a new call of I2C_Read. Should
 *                  enter interrupt where data is received from the i2c bus
 *
 * Overview:        Reads I2C data from the bus.
 *
 * Note:            This function should be called after a write command
 *                  
 *************************************************************************************/

 uint16_t I2C_Read(uint8_t * data, uint16_t length, uint8_t slave_address)
{ 
  uint8_t recycle = 0 , index=0; 
  uint8_t rx_buf_size=0;    // For debugging 
  SINGLE_BYTE_RX = 0;

/********************************************************************************
 * Wait for UART to not be busy.  If it takes too long return an error 
 *******************************************************************************/
 if(Wait_for_I2C_Not_Busy())
 {
   return ERROR_I2C_BUSY;
 }

/********************************************************************************
 * Set up registers to read data from the bus 
 *******************************************************************************/
  UCB0I2CSA = slave_address;                    // Set the slave address
  if (length==1)
    {SINGLE_BYTE_RX = 1;}                        // Stop not sent in ISR when only sending 1 byte
  RXCounter = length;                         /* Set global with total bytes to be RX'd,
                                                *  subtracting 1 is needed to adjust for the 
                                                *  fact that ISR considers 0 the last byte */  
  UCB0CTL1 &= ~UCTR;                            // Set RX mode 
  UCB0CTL1 |= UCTXSTT;                          // I2C start condition
  if (SINGLE_BYTE_RX) {
    while (UCB0CTL1 & UCTXSTT);                 // Start condition sent?
    UCB0CTL1 |= UCTXSTP;                        // I2C stop condition
    }
  __bis_SR_register(CPUOFF + GIE);               // Enter LPM0 w/ interrupts
                                                // Remain in LPM0 until all data
                                                // is RX'd

/********************************************************************************
 * Wait for UART to not be busy.  If it takes too long return an error 
 *******************************************************************************/

 if( Wait_for_I2C_Not_Busy() )
 {
   return ERROR_I2C_BUSY;
 }

/********************************************************************************
 * RX'd bytes are available in I2C_RX_buf, return only the number of bytes 
 * requested by length, if there are more recycle them. 
 *******************************************************************************/ 
 rx_buf_size =rbuffer_count(&I2C_RX_buf);   
 //debug_printf("%s%hu%s", "RX_buf size before emptying: ", rx_buf_size, "\n");

 while(rbuffer_count(&I2C_RX_buf)){
  // Get the first length bytes from the i2c_rx_buffer
  // JGM: if condition was (index <=length), this means we're gonna go out of bounds? 
  rx_buf_size = rbuffer_count(&I2C_RX_buf);
  //debug_printf("%s%hu%s", "RX_buf size: ", rx_buf_size, "\n");
  //debug_printf("%s%u%s", "Index: ", index, "\n");
  if (index<length)
    data[index] = rbuffer_getchar(&I2C_RX_buf);
  
  // if there are any remaining bytes in the buffer clear them out
  else 
    recycle = rbuffer_getchar(&I2C_RX_buf); 

  index++;
 }
 //debug_printf("%s", "\n");
 return I2C_NO_ERROR;
}


/*************************************************************************************
 * Function:        void I2C_IsrTX_Master (void)
 *
 * PreCondition:    I2C_Init_Master() ran and Master Tx Interrupt triggered
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When the Tx interrupt is trigger, this functions loads the next
 *                  character in the buffer into the I2C output buffer and tx the 
 *                  next character, at the last character it sends a stop 
 *
 * Note:            This function should not be used on its own put as part of a write
 *                  operation on the I2C bus
*************************************************************************************/

void I2C_IsrTX_Master (void)
{
/********************************************************************************
 * If the ring buffer loaded with bytes to transmit still has bytes left in it
 * then load UCB0TXBUF with the next byte
 *******************************************************************************/  
  if (rbuffer_count(&I2C_TX_buf))                         // Check TX byte counter
  {
    UCB0TXBUF = rbuffer_getchar(&I2C_TX_buf);             // Load TX buffer
  }
/********************************************************************************
* When the buffer is empty and the last byte has been transmitted send stop and
* clear the tx interrupt flag. 
*******************************************************************************/  
  else
  {
    UCB0CTL1 |= UCTXSTP;                                  // I2C stop condition
    IFG2 &= ~UCB0TXIFG;                                   // Clear USCI_B0 TX int flag
  }
}
  
/*************************************************************************************
 * Function:        void I2C_IsrRX_Master (void)
 *
 * PreCondition:    I2C_Init_Master() ran and Master Rx Interrupt triggered
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When the Rx interrupt is trigger, this functions loads the next
 *                  character in the buffer into the I2C output ring buffer and Rx the 
 *                  next character. Changes ISR behavior for single vs. multiple byte
 *                  Reception. 
 *
 * Note:            This function should not be used on its own put as part of a write
 *                  operation on the I2C bus. Magic not logic following if statements...
*************************************************************************************/

void I2C_IsrRX_Master(void)
{
/********************************************************************************
* Drive ISR behavior in the case of only single byte to RX 
*******************************************************************************/  
  if (SINGLE_BYTE_RX) {
    rbuffer_putchar(&I2C_RX_buf, UCB0RXBUF);            // Move RX data to address PRxData
    }
/********************************************************************************
* Drive ISR behavior in the case of only multiple bytes to RX (counts from 0...)
*******************************************************************************/  
  else{    
    RXCounter=RXCounter-1;                                          // Decrement RX byte counter
    if (RXCounter)
      {
      rbuffer_putchar(&I2C_RX_buf, UCB0RXBUF);            // Move RX data to address PRxData
      if (RXCounter == 1)                                 // Only one byte left?
        {
        UCB0CTL1 |= UCTXSTP;                             // Generate I2C stop condition
        //rbuffer_putchar(&I2C_RX_buf, UCB0RXBUF);         /* This line is nonsense for whatever crazy reason I can't discern why 
                                                          //* rbuffer_putchar is skipped in the if(RXCounter) when RXcounter==1... */
        }
      }
    else
      {
       rbuffer_putchar(&I2C_RX_buf, UCB0RXBUF);           // Move final RX data to PRxData
      }
    }
}

/******************************************************************************
*
* MSP430 I2C Master RX and TX interrupt
* 
* Note : USCIAB0TX_VECTOR handles both RX and TX interrupts despite it  having 
*        TX in the name of the vector. Very Counterintuitive. 
*
******************************************************************************/
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
 { 
 uint8_t interrupt_state; 
 // Find out if TX, RX, or both interrupts are high
 interrupt_state = (IFG2 & (UCB0RXIFG+UCB0TXIFG));
  
 switch(interrupt_state) {

  // Case: TXBuf is ready to accept a new byte
  case(UCB0TXIFG):
    // Transmit byte(s)
    I2C_IsrTX_Master(); 
    __bic_SR_register_on_exit(CPUOFF);                     // Exit LPM0
    break; 

  // Case: UCB0RXBUF is full and ready to be emptied 
  case(UCB0RXIFG):
    // Receive byte(s)
    I2C_IsrRX_Master();
    __bic_SR_register_on_exit(CPUOFF);                     // Exit LPM0
    break;
  
  // Case: either or neither RX or TX are high  
  default:
    ERROR_I2C_ISR_CONFLICT= interrupt_state;
    __bic_SR_register_on_exit(CPUOFF);                     // Exit LPM0
    break;
  }
 }


