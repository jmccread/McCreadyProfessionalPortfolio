#include "driverlib.h"
#include "usci_b_i2c_functionality_drivers.h"
#include "config.h"
#include "main.h"

// Declare globals for interrupts and declare a shared i2c device (maybe this will work)
usci_b_i2c_t i2c_device;
uint8_t transmitData[I2C_MAX_BUFFER_LENGTH];
uint16_t TXLENGTH;
uint8_t transmitCounter = 0;

uint8_t receiveBuffer[I2C_MAX_BUFFER_LENGTH];
uint8_t *receiveBufferPointer;
uint8_t receiveCount = 0;

/******************************************************************************
*
master_init(): 
function to initialize uscb_Bx for i2c single master with a specific slave functionality. 
Assumes SMCLK as clock source and 400 KBS data rate, you have to know the clock 
frequency unfortunately, the unified clock system is not implimented on the 
msp430f2618. 

The pin configuration must be done outside of this function. 

i2c_device is a pointer to a struct of type usci_b_i2c_t which is defined in usci_b_i2c.h. 

USCI_BX_BASE is either USCI_B0_BASE or USCI_B1_BASE
*
******************************************************************************/
void master_init(uint32_t USCI_Bx_BASE,
                uint32_t DESIRED_I2C_CLK_F)
	{	
        //Initialize Master, use SMCLK as default change in project if necessary. 
        USCI_B_I2C_masterInit(&i2c_device,
                              USCI_Bx_BASE,
                              USCI_B_I2C_CLOCKSOURCE_SMCLK,
                              DESIRED_I2C_CLK_F,
                              USCI_B_I2C_SET_DATA_RATE_400KBPS
                              );
        
        //Enable I2C Module to start operations
        USCI_B_I2C_enable(&i2c_device);
        
        //Enable TX interrupt
        USCI_B_I2C_clearInterruptFlag(&i2c_device,
                                      USCI_B_I2C_TRANSMIT_INTERRUPT);
									  
        USCI_B_I2C_enableInterrupt(&i2c_device,
                                   USCI_B_I2C_TRANSMIT_INTERRUPT);

        //Enable master Receive interrupt
        USCI_B_I2C_clearInterruptFlag(&i2c_device,
                                      USCI_B_I2C_RECEIVE_INTERRUPT);
									  
        USCI_B_I2C_enableInterrupt(&i2c_device,
                                   USCI_B_I2C_RECEIVE_INTERRUPT); 

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
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Sends the command of the specified length, to the specified slave
 *                  device over I2C.
 *
 * Note:            This function assumes the use of USCIB0, changes should be made to
 *					configurations to adjust for USCIB1 if it is implemented. 
*************************************************************************************/
uint8_t I2C_Write(uint8_t* command, uint8_t length, uint8_t slave_address)
{
/********************************************************************************
 * Wait for UART to not be busy.  If it takes too long return an error 


/********************************************************************************
 * Verify that the data to be sent fits in the buffer 
 *******************************************************************************/
  if (length > I2C_MAX_BUFFER_LENGTH) 
  {
    return ERROR_I2C_DATA_TOO_LONG;
  }

/********************************************************************************
 * Set up registers to write data to the bus 
 *******************************************************************************/
  //Specify slave address						
  USCI_B_I2C_setSlaveAddress(&i2c_device, slave_address);	

  //Set in transmit mode
  USCI_B_I2C_setMode(&i2c_device,
                     USCI_B_I2C_TRANSMIT_MODE);

  //Enable I2C Module to start operations
  USCI_B_I2C_enable(&i2c_device);
  
  //Enable transmit Interrupt
  USCI_B_I2C_clearInterruptFlag(&i2c_device,
                                USCI_B_I2C_TRANSMIT_INTERRUPT);

  USCI_B_I2C_enableInterrupt(&i2c_device,
                             USCI_B_I2C_TRANSMIT_INTERRUPT);

  // Copy command into global buffer, assign length to global  
  memcpy(transmitData, command, length );           
  TXLENGTH = length; 
  
  //Load TX byte counter
  transmitCounter = 1;

  //Initiate start and send first character
  USCI_B_I2C_masterMultiByteSendStart(&i2c_device,
                                      transmitData[0]);
									  
  //Enter LPM0 with interrupts enabled
  //__bis_SR_register(LPM0_bits + GIE);
 
/********************************************************************************
 * Wait for UART to not be busy.  If it takes too long return an error
 *******************************************************************************/
  /*
  if(Wait_for_I2C_Ready())
  {
   return ERROR_I2C_BUSY;
  }
  */
 // TO DO: CHECK TO SEE IF NACK AND ARB INTERRUPTS NEED TO BE ENABLED
 // TO DO: FIX THIS ITS EXITING TOO EARLY 
/********************************************************************************
 * Was transmission successful?  Check for NACK 
 *******************************************************************************/
 if((UCB0STAT & UCNACKIFG) != 0x08)
 {
  //If the I2C flag is not set then the communication was successful
  return I2C_NO_ERROR;
 }
 else
 {
  //Reset the Flag, will also reset on a new start condition
  UCB0STAT &= ~UCNACKIFG;

  //If the I2C flag was set then the communication was not successful 
  return ERROR_I2C_NACK_RECEIVED;
 }

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
 * Function:        uint8_t I2C_Read(uint8_t *data, uint16_t length, uint8_t slave_address)
 *
 * PreCondition:    I2C_Init_Master() and I2C_Write ran
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        
 *
 * Note:            This function should not be used on its own put as part of a write
 *                  or read operation on the I2C bus
*************************************************************************************/
uint8_t I2C_Read(uint8_t *data, uint8_t length, uint8_t slave_address)
{
/********************************************************************************
 * Wait for UART to not be busy.  If it takes too long return an error 
 *******************************************************************************/
 if(Wait_for_I2C_Ready())
 {
   return ERROR_I2C_BUSY;
 }
   
/********************************************************************************
 * Verify that the data to be sent fits in the buffer 
 *******************************************************************************/
  if (length > I2C_MAX_BUFFER_LENGTH) 
  {
    return ERROR_I2C_DATA_TOO_LONG;
  }

/********************************************************************************
 * Set up registers to read data from the bus 
 *******************************************************************************/
   //Set in receive mode
  USCI_B_I2C_setMode(&i2c_device,
                     USCI_B_I2C_RECEIVE_MODE);
					 
  //Specify slave address						
  USCI_B_I2C_setSlaveAddress(&i2c_device, 	  
							slave_address);	
  
  //Setup receive globals  
  receiveBufferPointer = (uint8_t*)receiveBuffer;
  receiveCount = length;

  //Initialize multi byte reception
  USCI_B_I2C_masterMultiByteReceiveStart(&i2c_device);

  //Enter low power mode 0 with interrupts enabled.
  __bis_SR_register(LPM0_bits + GIE);
  __no_operation();	
							
/********************************************************************************
 * Wait for UART to not be busy.  If it takes too long return an error 
 *******************************************************************************/
 if(Wait_for_I2C_Ready())
 {
   return ERROR_I2C_BUSY;
 }
 
 // If rx is complete return no error, access data using receiveBuffer
 return I2C_NO_ERROR;
}


// TO DO: IS THERE A BETTER WAY OF ACCOMPLISHING THIS THAN LOOPING? TI API FUNCTION? 
/*************************************************************************************
 * Function:        uint8_t Wait_for_I2C_Ready(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Waits for the USCI_B0_I2C BUS to be ready
 *
 * Note:            This function should not be used on its own put as part of a write
 *                  or read operation on the I2C bus
*************************************************************************************/
uint8_t Wait_for_I2C_Ready(void)
{
  uint16_t ui16_I2C_counter = 0;             /* I2C busy counter                    */
  ui16_I2C_counter = 0;                      /* Clear the counter                   */

  //Wait for UART to not be busy.  If it takes too long return an error 
  for(ui16_I2C_counter = 0; ui16_I2C_counter < 0xFFFF;ui16_I2C_counter++){
   if (UCB0STAT & UCBBUSY)
	{
	return I2C_NO_ERROR;
	}
  }

  return ERROR_I2C_BUSY;                 /*If the counter overflows return error*/
}
	
/******************************************************************************
*
Blah, interrupts, eternal damnation... 
*
******************************************************************************/
/*
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCI_B0_TX_ISR(void)
{
        if (IFG2 & UCB0TXIFG)
        {
                //Check TX byte counter
                if (transmitCounter < TXLENGTH) {
                        //Initiate send of character from Master to Slave
                        USCI_B_I2C_masterMultiByteSendNext(&i2c_device,
                                                           transmitData[transmitCounter]);

                        //Increment TX byte counter
                        transmitCounter++;
                } else {
                        //Initiate stop only
                        USCI_B_I2C_masterMultiByteSendStop(&i2c_device);

                        //Clear master interrupt status
                        USCI_B_I2C_clearInterruptFlag(&i2c_device,
                                                      USCI_B_I2C_TRANSMIT_INTERRUPT);

                        //Exit LPM0 on interrupt return
                        //__bic_SR_register_on_exit(LPM0_bits);
                }
        }  
}
*/
// TO DO: SEE IF THIS ALL BREAKS WHEN ONLY 1 BYTE IS SENT (PROBABLY) 
/******************************************************************************
*
Blah, interrupts, eternal damnation... 
*
******************************************************************************/
/*
#pragma vector =  USCIAB0RX_VECTOR
__interrupt void USCI_B0_RX_ISR(void)
{
        if (IFG2 & UCRXIFG)
        {
                //Decrement RX byte counter
                receiveCount--;

                if (receiveCount) {
                        if (receiveCount == 1) {
                                //Initiate end of reception -> Receive byte with NAK
                                *receiveBufferPointer++ =
                                        USCI_B_I2C_masterMultiByteReceiveFinish(
                                                &i2c_device);				
                        }else  {
                                //Keep receiving one byte at a time
                                *receiveBufferPointer++ = USCI_B_I2C_masterMultiByteReceiveNext(
                                        &i2c_device);
                        }
                }else  {
                        //Receive last byte
                        *receiveBufferPointer = USCI_B_I2C_masterMultiByteReceiveNext(
                                &i2c_device
                                );
                        __bic_SR_register_on_exit(LPM0_bits);
                }
        }
}
*/
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCI_B0_TX_ISR(void)
{
        switch (IFG2) {
        case USCI_I2C_UCTXIFG:
        {
                //Check TX byte counter
                if (transmitCounter < TXLENGTH) {
                        //Initiate send of character from Master to Slave
                        USCI_B_I2C_masterMultiByteSendNext(&i2c_device,
                                                           transmitData[transmitCounter]
                                                           );

                        //Increment TX byte counter
                        transmitCounter++;
                } else {
                        //Initiate stop only
                        USCI_B_I2C_masterMultiByteSendStop(&i2c_device);

                        //Clear master interrupt status
                        USCI_B_I2C_clearInterruptFlag(&i2c_device,
                                                      USCI_B_I2C_TRANSMIT_INTERRUPT);
                        /*
                        //Exit LPM0 on interrupt return
                        __bic_SR_register_on_exit(LPM0_bits);*/
                }
                break;
        }
        }
}
