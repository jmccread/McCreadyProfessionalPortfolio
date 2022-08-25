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
 * date:	07/30/2014
 */

/******************************************************************************
* UART.h
******************************************************************************/
#ifndef __UART_h
#define __UART_h

/********************************************************************************
*
* Includes
*
 *******************************************************************************/
#include "stdint.h"
#include "driverlib.h" 
#include "msp430.h"
#include "usci_a_uart.h"
#include "config.h"
#include "fletcher16.h"
/********************************************************************************
*
* Definitions
*
 *******************************************************************************/

 /********************************************************************************
* Macros for UART errors 
 *******************************************************************************/
#define UART_NO_ERROR                           0x0000
#define ERROR_UART_BUSY                         0x0001

/********************************************************************************
* Macros for sending numbers via uart
* These macros are to be used to send a series of numbers of a single type over UART
* so they can be easily decoded after reception on the other end of a serial line
* 
* Each UART transmission of this type should begin with three bytes
*   1. UART_START_BYTE
*   2. DATA_TYPE_BYTE
*   3. NUM_IN_SEQUENCE -- amount of numbers of type being sent
*
* Each UART Transmission should end with a checksum (simple XOR of all bytes 
* Prior to the checksum there will be the ASCII character *) 
*
* Multiple transmissions may be sent using the separator which should directly 
* follow the checksum bytes(?) and indicates a new transmission. 
 *******************************************************************************/
#define UART_START_BYTE        0x02    // ASCII for start of text 
#define UART_CHAR              0x00
#define UART_UINT8             0x01
#define UART_UINT16            0x02
#define UART_UINT32            0x03
#define UART_INT8              0x04
#define UART_INT16             0x05
#define UART_INT32             0x06
#define UART_FLOAT             0x07
#define UART_STOP_BYTE         0x03    // ASCII for end of text 
#define UART_SEPARATOR         0x1F    // ASCII Unit separator 

/********************************************************************************
* Variables 
 *******************************************************************************/
extern rbuffer_t UART_RX;
extern rbuffer_t UART_TX; 
extern usci_a_uart_t UART_A1;

/********************************************************************************
*
* Function Prototypes and ISRs 
*
 *******************************************************************************/
uint8_t start_device_uart(uint32_t baud_rate); 
uint8_t Wait_for_UART_Not_Busy(usci_a_uart_t *in_ptr, uint32_t master_speed);

uint8_t uart_transmit_float(usci_a_uart_t *in_ptr, float * data, uint8_t ID, uint8_t length);
uint8_t uart_transmit_uint32(usci_a_uart_t *in_ptr, uint32_t * data, uint8_t ID, uint8_t length);
uint8_t uart_transmit_int32(usci_a_uart_t *in_ptr, int32_t * data, uint8_t ID, uint8_t length);
uint8_t uart_transmit_uint16(usci_a_uart_t *in_ptr, uint16_t * data, uint8_t ID, uint8_t length);
uint8_t uart_transmit_int16(usci_a_uart_t *in_ptr, int16_t * data, uint8_t ID, uint8_t length);
uint8_t uart_transmit_uint8(usci_a_uart_t *in_ptr, uint8_t * data, uint8_t ID, uint8_t length);
uint8_t uart_transmit_int8(usci_a_uart_t *in_ptr, int8_t * data, uint8_t ID, uint8_t length);

uint8_t uart_recieve_float(usci_a_uart_t *in_ptr, uint32_t mclk_speed, float * data, uint8_t ID); 
uint8_t uart_recieve_uint32(usci_a_uart_t *in_ptr, uint32_t mclk_speed, uint32_t * data, uint8_t ID); 
uint8_t uart_recieve_int32(usci_a_uart_t *in_ptr, uint32_t mclk_speed, int32_t * data, uint8_t ID); 
uint8_t uart_recieve_uint16(usci_a_uart_t *in_ptr, uint32_t mclk_speed, uint16_t * data, uint8_t ID); 
uint8_t uart_recieve_int16(usci_a_uart_t *in_ptr, uint32_t mclk_speed, int16_t * data, uint8_t ID);
uint8_t uart_recieve_uint8(usci_a_uart_t *in_ptr, uint32_t mclk_speed, uint8_t * data, uint8_t ID); 
uint8_t uart_recieve_int8(usci_a_uart_t *in_ptr, uint32_t mclk_speed, int8_t * data, uint8_t ID);

__interrupt void USCI_A1_TX_ISR(void); 
__interrupt void USCI_A1_RX_ISR(void); 
#endif /*_UART_h

