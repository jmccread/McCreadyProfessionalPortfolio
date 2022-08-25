// I2Cdev library collection - Main I2C device class header file
// Abstracts bit and byte I2C R/W functions into a convenient class
// 6/9/2012 by Jeff Rowberg <jeff@rowberg.net>

/******************************************
 *  ____    ____   ____  ____   _____     *	
 * |_   \  /   _| |_  _||_  _| |_   _|    *
 *   |   \/   |     \ \  / /     | |      *
 *   | |\  /| |      > `' <      | |   _  *
 *  _| |_\/_| |_   _/ /'`\ \_   _| |__/ | *	
 * |_____||_____| |____||____| |________| *	
 *                                        *	
 ******************************************/

/*
* adapted by: 	Joshua McCready 
* email:	jmccread@umich.edu
* date:	07/23/2014
*/

#ifndef _I2CDEV_H_
#define _I2CDEV_H_
/******************************************************************************
* Includes 
******************************************************************************/
#include "I2C.h"



// -----------------------------------------------------------------------------
// I2C interface implementation setting
// -----------------------------------------------------------------------------
#define I2CDEV_IMPLEMENTATION       I2CDEV_MSP430

// -----------------------------------------------------------------------------
// I2C interface implementation options
// -----------------------------------------------------------------------------
#define I2CDEV_MSP430				4 // Experimental MSP430 (MSP430F2618) implementation from Andreas Zoellner

// 1000ms default read timeout (modify with "I2Cdev::readTimeout = [ms];")
#define I2CDEV_DEFAULT_READ_TIMEOUT     0



uint8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout);
uint8_t readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout);
uint8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout);
uint8_t readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout);
uint8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout);
uint8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout);
uint8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout);
uint8_t readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout);

int writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
int writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
int writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
int writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
int writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
int writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
int writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
int writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

extern uint16_t readTimeout;


#endif /* _I2CDEV_H_ */
