/******************************************
 *  ____    ____   ____  ____   _____     *	
 * |_   \  /   _| |_  _||_  _| |_   _|    *
 *   |   \/   |     \ \  / /     | |      *
 *   | |\  /| |      > `' <      | |   _  *
 *  _| |_\/_| |_   _/ /'`\ \_   _| |__/ | *	
 * |_____||_____| |____||____| |________| *	
 *                                        *	
 ******************************************/
 
/**
 * author: 	Joshua McCready 
 * email:	jmccread@umich.edu
 * date:	08/12/2014
 */

/******************************************************************************
*
* TCA9548A.c
* Source file for the tca9548a which is an 8 channel i2c switched with reset
* First implimented on the MCU of Project Strato's single axis reaction wheel
* payload 
*
******************************************************************************/

/******************************************************************************
*
* Includes 
*
******************************************************************************/
#include "tca9548.h"
#include "main.h"

/******************************************************************************
*
* Declarations  
*
******************************************************************************/

 /*************************************************************************************
 * Function:        uint8_t tca9548_channel_select(uint8_t channels, uint8_t address)
 *
 * PreCondition:    I2C has been properly configured with I2C_INIT_MASTER and I2c pins
 *                  have been configured 
 *
 * Input:           Parameters: 
 *                    - uint8_t channels:
 *                      Bit0 = Channel 0 enabled
 *                      Bit1 = Channel 1 enabled
 *                      ...
 *                      Bit8 = Channel 8 enabled
 *                    - uint8_t address -- device address 
 *
 * Output:          uint8_t e: any error code resulting from i2c communication 
 *
 * Side Effects:   Enables different i2c buses 
 *
 * Overview:       Writes channels to the address of the device
 *
 * Note:           None 
 *                  
 *************************************************************************************/
uint8_t tca9548_channel_select(uint8_t channels, uint8_t address){
  e.tca = I2C_Write(&channels, 0x01, address);
  return e.tca; 
}