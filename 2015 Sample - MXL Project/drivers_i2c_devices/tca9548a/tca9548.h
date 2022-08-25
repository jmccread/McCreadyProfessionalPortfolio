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
* TCA9548A.h
* Header file for the tca9548a which is an 8 channel i2c switched with reset
* First implimented on the MCU of Project Strato's single axis reaction wheel
* payload 
*
******************************************************************************/

#ifndef __tca9548a_h
#define __tca9548a_h

/******************************************************************************
*
* Includes 
*
******************************************************************************/
#include "i2c.h"
#include "i2cdev.h"
#include "stdint.h"
/******************************************************************************
*
* Definitions 
*
******************************************************************************/
// Device Addresses -- hardware configurable from inputs A0-A2
#define TCA9548A_Addr_LLL        0x70
#define TCA9548A_Addr_LLH        0x71
#define TCA9548A_Addr_LHL        0x72
#define TCA9548A_Addr_LHH        0x73
#define TCA9548A_Addr_HLL        0x74
#define TCA9548A_Addr_HLH        0x75
#define TCA9548A_Addr_HHL        0x76
#define TCA9548A_Addr_HHH        0x77
#define TCA9548A_Addr_Default    TCA9548A_Addr_LLL

/* Following an ACK of the device address the command byte that selects
enabled channels (multiple can be enabled at once, channel 0 corresponds
to bit 0 of the command byte, channel 1 to bit 1, and so on and so forth*/

/******************************************************************************
*
* Declarations and Macros  
*
******************************************************************************/
uint8_t tca9548_channel_select(uint8_t channels, uint8_t address);
#endif 
