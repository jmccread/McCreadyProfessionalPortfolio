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
* pca9543a.h
*
******************************************************************************/

#ifndef __pca9543a_h
#define __pca9543a_h

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

/******************************************************************************
* Device Definitions and Register Map
******************************************************************************/
// Device Addresses -- hardware configurable from inputs A0-A2
#define PCA9543A_Addr_LLL        0x20
#define PCA9543A_Addr_LLH        0x21
#define PCA9543A_Addr_LHL        0x22
#define PCA9543A_Addr_LHH        0x23
#define PCA9543A_Addr_HLL        0x24
#define PCA9543A_Addr_HLH        0x25
#define PCA9543A_Addr_HHL        0x26
#define PCA9543A_Addr_HHH        0x27
#define PCA9543A_Addr_Default    PCA9543A_Addr_LLL

// Internal Control Register and structure
/* 
* Reflects the  incoming high/low level of a IO port regardless of configuration
* Writing to this register does nothing, it is read only.
*/
#define PCA9543A_RA_IP           0x00     // Input port reg

/* -- MAY NOT BE CORRECT --
* Reflects the out going high/low level of pins defined as outputs,
* Default is 0xFF meaning that all pins are initially defined as inputs 
*/
#define PCA9543A_RA_OP           0x01     // Output port reg

/* -- Does Polarity correspond to in/out? -- 
* Writing 1 to a bit in this register flips the port pin polarity, writing 0 does nothing
*/
#define PCA9543A_RA_PI           0x02     // Polarity Inversion reg

/* 
* 1 corresponds to input, 0 to output
*/
#define PCA9543A_RA_CFG          0x03     // Configuration

// Errors
  //TO DO: Define all the errors in self check!!!!

/******************************************************************************
*
* Declarations and Macros  
*
******************************************************************************/
uint8_t pca9543a_CFG_IO(uint8_t channels, uint8_t address); 
uint8_t pca9543a_Polarity_Invert(uint8_t channels, uint8_t address);
uint8_t pca9543a_Set_Output(uint8_t channels, uint8_t address);
uint8_t pca9543a_Read_Input_Levels(uint8_t *levels, uint8_t address);
uint8_t pca9543a_Read_Output_Levels(uint8_t *levels, uint8_t address);
uint8_t pca9543a_Self_Test(uint8_t output_level, uint8_t address); 
#endif 
