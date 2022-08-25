/******************************************************************************
*
usci_b_i2c_functionality_drivers.c
*
******************************************************************************/
#ifndef USCI_B_I2C_FUNCTIONALITY_DRIVERS_H
#define USCI_B_I2C_FUNCTIONALITY_DRIVERS_H

/********************************************************************************
 * Includes 
 *******************************************************************************/
#include "driverlib.h"

/********************************************************************************
 * Definitions
 *******************************************************************************/
#define I2C_MAX_BUFFER_LENGTH                  255        /* I2C Buffer Size   */

/********************************************************************************
 * I2C Errors
 *******************************************************************************/
#define I2C_NO_ERROR                           0x00
#define ERROR_I2C_BUSY                         0x01
#define ERROR_I2C_DATA_TOO_LONG                0x02
#define ERROR_I2C_NACK_RECEIVED                0x03
#define ERROR_I2C_LOSTARB_RECEIVED             0x04

/** V A R I A B L E S ***************************************************************/
extern usci_b_i2c_t i2c_device;
extern uint8_t receiveBuffer[I2C_MAX_BUFFER_LENGTH];                             


/** P R O T O T Y P E S **************************************************************/
// Initializes the I2C bus 
void master_init( uint32_t USCI_Bx_BASE, uint32_t DESIRED_I2C_CLK_F);
// Writes to I2C				  
uint8_t I2C_Write(uint8_t* command, uint8_t length, uint8_t slave_address);
// Reads I2C data
uint8_t I2C_Read(uint8_t *data, uint8_t length, uint8_t slave_address);
// Waits for I2C to be ready 
uint8_t Wait_for_I2C_Ready(void); 

#endif
