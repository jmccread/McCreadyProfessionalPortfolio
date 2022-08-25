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
 * date:	07/14/2014
 */

#ifndef __common_i2c_ops_h
#define __common_i2c_ops_h

// Function Declarations
void read_internal_reg(uint8_t int_reg, uint8_t length, uint8_t * result, uint8_t dev_addr); 
void write_internal_reg(uint8_t int_reg, uint8_t data, uint8_t dev_addr); 
 
#endif 

