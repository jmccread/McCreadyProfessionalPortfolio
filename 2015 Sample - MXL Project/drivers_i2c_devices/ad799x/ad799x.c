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
 * date:	07/01/2014
 */
 

 #include "I2C.h"
 #include "ad799x.h"
 #include "task_ad799x.h"
 #include "common_i2c_ops.h"


 /*************************************************************************************
 * Function:        command_mode_read(uint8_t config, uint8_t dev_addr, uint16_t * result[4])      
 *
 * PreCondition:    Configure pins for I2C and run I2C_Init_Master with correct inputs
 *
 * Input:           Parameters:
 *                    - channels: 8 bit number containing the address of the channels 
 *                      that are to be read, see ad799x.h
 *                    - dev_addr: address of the particular ad799x chip being talked to
 *                    - * channel_int: Location of size 4 array of uin16_t where the 
 *                      results of conversions may be put. 
 * Output:           None
 *
 * Side Effects:    Writes to result in calling function
 *
 * Overview:       Reads each channel of the ad7994 individually and writes the 12 bit 
 *                 data to the passed in array of unsigned int
 *
 * Note:          This was a huge pain in the butt because the adc cannot do multiple
 *                channel reads at once even though the datasheet says so. 
 *                  
 *************************************************************************************/
uint8_t command_mode_read(uint8_t channels, uint8_t address, uint16_t * channel_int) 
  { 
    // Have to declare variables so that I can point to them with I2C_Write,
    // kinda ineffiecent but okay for now. 
    uint8_t ch1 = CH1; 
    uint8_t ch2 = CH2; 
    uint8_t ch3 = CH3; 
    uint8_t ch4 = CH4; 
    uint8_t raw_byte[8];
    uint16_t channel_ints[4];
    uint8_t e= 0x00; 
 
   
   
    // Write data and read data 1 channel at a time because otherwise everything breaks.
    // Process 16 bit data down to 12 bit integer values according to pg 20  of 
    // the data sheet about Conversion Result Registers 
    if (channels & CH1) {
      e= I2C_Write(&ch1, 1, address);
      e= readBytes(address, CONV_RES, 2, &(raw_byte[0]), 0); 
      channel_int[0] =(uint16_t)(raw_byte[0]&(~0xF0))<<8 | raw_byte[1];
      channel_ints[0] =(uint16_t)(raw_byte[0]&(~0xF0))<<8 | raw_byte[1];
      }
    if (channels & CH2) {
      e= I2C_Write(&ch2, 1, address);
      e= readBytes(address, CONV_RES, 2, &(raw_byte[2]), 0);
      channel_int[1] = (uint16_t)(raw_byte[2]&(~0xF0))<<8 | raw_byte[3];  
      channel_ints[1] = (uint16_t)(raw_byte[2]&(~0xF0))<<8 | raw_byte[3]; 
      }
    if (channels & CH3) {
      e= I2C_Write(&ch3, 1, address);
      e= readBytes(address, CONV_RES, 2, &(raw_byte[4]), 0);
      channel_int[2] = (uint16_t)(raw_byte[4]&(~0xF0))<<8 | raw_byte[5];
      channel_ints[2] = (uint16_t)(raw_byte[4]&(~0xF0))<<8 | raw_byte[5];
      }
    if (channels & CH4) {
      e= I2C_Write(&ch4, 1, address);
      e= readBytes(address, CONV_RES, 2, &(raw_byte[6]), 0);
      channel_int[3] = (uint16_t)(raw_byte[6]&(~0xF0))<<8 | raw_byte[7];
      //channel_ints[3] = (uint16_t)(raw_byte[6]&(~0xF0))<<8 | raw_byte[7];
      }
    return e; 
    }