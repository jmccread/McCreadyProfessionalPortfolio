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
 * date:	07/03/2014
 */

// TO DO: Remove this set of files and transitions to using i2cdev.
#include "i2c.h"
#include "msp430.h"
#include "stdint.h"
/*************************************************************************************
 * Function:          uint8_t read_internal_reg(uint8_t int_reg)        
 *
 * PreCondition:      None
 *
 * Input:             Parameters:
 *                      -int_reg: 8 bit location of data to be requested from device
 *                      -length: expected length of received information
 *                      -*result: Pointer to where the received bytes will go
 *                      -dev_addr: address of the i2c device you want to read 
 *                      
 * Output:           
 *
 * Side Effects:      
 *
 * Overview:          Reads 1 or two bytes after telling the device where to read from
 *
 * Note:	      

 *                  
 *************************************************************************************/
void read_internal_reg(uint8_t int_reg, uint8_t length, uint8_t * result, uint8_t dev_addr)  
 { 
  // Write to desired internal register before read
  I2C_Write(&int_reg, 0x01, dev_addr);  
	 
         
  // Read from address pointer register
  I2C_Read(result, length, dev_addr); 

 }

 /*************************************************************************************
 * Function:       write_internal_reg(uint8_t int_reg, uint8_t data, uint8_t dev_addr)         
 *
 * PreCondition:    None  
 *
 * Input:           Parameters:
 *                    - int_reg:  moves internal address pointer to int_reg
 *                    - data:     number written to internal register
 *                    - dev_addr: address of the ad799x chip being used 
 *
 * Output:          None   
 *
 * Side Effects:    Changes address pointer to int_reg and writes to that location
 *
 * Overview:        This function uses the existing i2c libraries to write to an 
 *                  one of the internal registers on the adf799x chip 
 *
 * Note:            None
 *                  
 *************************************************************************************/
 uint8_t write_internal_reg(uint8_t int_reg, uint8_t data, uint8_t dev_addr)  {
  uint8_t e=0x00;
  // Write to desired internal register before write data to said register
  e =I2C_Write(&int_reg, 0x01, dev_addr); // write to  configure register 
  e =I2C_Write(&data, 0x01, dev_addr); 
  return e; 
 }