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
* bus_isolation.c
* Manages the various modes of isolation needed to allow Cerebro to function 
*
******************************************************************************/

/******************************************************************************
*
* Includes 
*
******************************************************************************/
#include "ltc1393.h"
#include "main.h"
/******************************************************************************
*
* Declarations  
*
******************************************************************************/

/*************************************************************************************
 * Function:        uint8_t ltc1393_channel_select(uint8_t channels, uint8_t address)
 *
 * PreCondition:    I2C has been properly configured with I2C_INIT_MASTER and I2c pins
 *                  have been configured 
 *
 * Input:           Parameters: 
 *                    - channels: use #defined macros from ltc1393.h for channel selection
 *
 * Output:          uint8_t e: any error code resulting from i2c communication 
 *
 * Side Effects:   Enables different different ports on the multiplexer 
 *
 * Overview:       Writes channels to the address of the device
 *
 * Note:           None 
 *                  
 *************************************************************************************/
uint8_t ltc1393_channel_select(uint8_t channels, uint8_t address){
  e.ltc = I2C_Write(&channels, 0x01, address);
  return e.ltc;
}
