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
* bus_isolation.h
*
******************************************************************************/

#ifndef __bus_isolation_h
#define __bus_isolation_h

/******************************************************************************
*
* Includes 
*
******************************************************************************/
#include "ltc1393.h"
#include "pca9543a.h"
#include "tca9548.h"
#include "main.h"

/******************************************************************************
*
* Definitions 
*
******************************************************************************/
// CMD_I2C Address Space
#define PERIPH_ADC            0x21
#define MAGENTOMETER          0x1E
#define IO_E_B                0x3F
#define IO_E_A                0x3E
#define PRES_TEMP_SENS        0x77
#define GYRO                  0x69
#define ACC                   0x53
#define UART_MUX              0x4C
#define I2C_MUX               0x70

// Output (High or low) configurations for Input Output Expander A
#define A_PERIPHERAL_5_EN_VCC 0x02
#define A_PERIPHERAL_5_EN_I2C 0x01
#define A_GPS_UART_EN         0x04
#define A_XBEE_UART_EN        0x08
#define A_GYRO_XCL_PT_EN      0x10
#define A_MOTOR_CTRL_EN_VCC   0x40
#define A_MOTOR_CTRL_EN_I2C   0x20
#define A_FULL_PWR_EN         0x42

// Output (High or low) configurations for Input Output Expander B
#define B_PERIPHERAL_1_EN_VCC 0x02
#define B_PERIPHERAL_2_EN_VCC 0x08
#define B_PERIPHERAL_3_EN_VCC 0x20
#define B_PERIPHERAL_4_EN_VCC 0x80
#define B_PERIPHERAL_1_EN_I2C 0x01
#define B_PERIPHERAL_2_EN_I2C 0x04
#define B_PERIPHERAL_3_EN_I2C 0x10
#define B_PERIPHERAL_4_EN_I2C 0x40
#define B_FULL_PWR_EN         0xAA

// UART Multiplexing 

/* UART_DEVICE_CONNECT connects any devices (here Xbee and GPS module to both
   the MSP430 and STAMP at once, only one device may be commanded at once
   and all others should be isolated during uart communication */
#define UART_DEVICE_CONNECT   0x0B 

/* STAMP_MSP_Connect connects the stamp and msp430 which flips the RX and TX lines
   of the msp430 ad stamp so that they can talk to one another. */
#define STAMP_MSP_CONNECT     0x09

// I2C Multiplexing 
#define CMD_I2C_EN            0x01
#define TELEM_I2C_EN          0x02
#define I2C_DISABLE           0x00

/******************************************************************************
*
* Declarations and Macros  
*
******************************************************************************/
uint8_t init_isolation_state(void);
uint8_t power_IO_E_AB(void);
uint8_t IO_E_ENABLE(uint8_t config, uint8_t address);
uint8_t IO_E_DISABLE(uint8_t config, uint8_t address); 
uint8_t UART_Channel_Select(uint8_t channel);
uint8_t I2C_Channel_Select(uint8_t channel);

#endif 
