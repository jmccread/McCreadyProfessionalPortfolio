/************************************************************************************
 FileName:     I2C.c
 Dependencies: See INCLUDES section
 Processor:    MSP430
 Hardware:     The code is natively intended to be used on the 
	       Cubesat Kit Platform.  
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
        1) I2C Timeout set to 9 ms.  This is enough time to send 900 bytes via I2C at
           100 KHz.
************************************************************************************/
#ifndef I2C_H
#define I2C_H

/** INCLUDES ***********************************************************************/
#include <stdint.h>
#include <rbuffer.h>

/** CONFIGURATION *******************************************************************/

/********************************************************************************
 * General
 *******************************************************************************/
#define I2C_MAX_BUFFER_LENGTH                   255       /* I2C Buffer Size   */
#define OWN_ADDR                                0x00      /* FCPU address      */
#define I2C_TIMEOUT                             0x100000   /* I2C Timeout       */

/********************************************************************************
 * I2C Errors
 *******************************************************************************/
#define I2C_NO_ERROR                           0x00
#define ERROR_I2C_BUSY                         0x01
#define ERROR_I2C_DATA_TOO_LONG                0x02
#define ERROR_I2C_LOSTARB_RECEIVED             0x03
#define ERROR_I2C_NACK_RECEIVED                0xFF  
#define ERROR_I2C_TXIFG_TIMEOUT                0x04
#define ERROR_I2C_ADD_ACK_TIMEOUT	       0x05
extern uint8_t ERROR_I2C_ISR_CONFLICT;         //0x0006

/*****************************************************************************
*
* The following are values that can be passed to the dataRate parameter for
* functions: I2C_Init_Master().
*
*****************************************************************************/
#define USCI_B_I2C_SET_DATA_RATE_400KBPS                                 400000
#define USCI_B_I2C_SET_DATA_RATE_100KBPS                                 100000
#define USCI_B_I2C_SET_DATA_RATE_10KBPS                                  10000													   

/** S T R U C T U R E S *************************************************************/
extern rbuffer_t I2C_RX_buf;                    /* Viewable in calling function to 
                                                access RX'd data of */ 

/** V A R I A B L E S *************************************************************/

/** P R O T O T Y P E S **************************************************************/
/* Initializes the I2C bus                                                          */
void I2C_Init_Master(uint8_t selectClockSource,
                     uint32_t i2cClk,
                     uint32_t dataRate);

/* Writes to I2C                                                                    */
extern uint16_t I2C_Write(uint8_t* command, uint8_t length, uint8_t slave_address);

/* Waits for I2C to be busy                                                       */
extern uint8_t Wait_for_I2C_Not_Busy();
 
/* Reads I2C data                                                                   */
extern uint16_t I2C_Read(uint8_t * data, uint16_t length, uint8_t slave_address);

/* Transmits a character which I2C is ready                                         */
extern void I2C_IsrTX_Master (void);

/* Receives a character when the I2C is ready                                       */
extern void I2C_IsrRX_Master (void);

#endif
