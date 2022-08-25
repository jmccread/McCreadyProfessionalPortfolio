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
 * date:	07/23/2014
 */

/******************************************************************************
* config.h
* Description:
* Defines human readable macros for the pin configuration of the MCU board 
******************************************************************************/
#ifndef __CONFIG_H
#define __CONFIG_H

#include <msp430.h>         

/******************************************************************************
*
* Pin Map -- MCU board 
*
******************************************************************************/

/******************************************************************************
* Communication Protocol Related Macros 
******************************************************************************/
// SD card (SPI) UCB1
#define SD_DIR                P5DIR
#define SD_SEL                P5SEL
#define SD_OUT                P5OUT
#define SD_MISO               BIT2
#define SD_MOSI               BIT1
#define SD_CLK                BIT3
 
#define CS_DIR                P4DIR
#define CS_SEL                P4SEL
#define CS_OUT                P4OUT
#define CS_SD                 BIT4

// Define What UC Bus SD card is on
#define SD_UCB1               0x01

// I2C UCB0
#define I2C_DIR               P3DIR
#define I2C_SEL               P3SEL 
#define I2C_SEL2              P3SEL2 
#define I2C_OUT               P3OUT
#define PIN_SDA               BIT1
#define PIN_SCL               BIT2

// Define which UCXX bus is used for
#define I2C_UCB0              0x01
#define UART_USCI_A1          0x01   
//#define UART_USCI_A0          0x01

// UART 
#define CEREBRO_UART_A        UART_USCI_A0
//#define CEREBRO_UART_B        UART_USCI_A1
#define UART_DIR              P3DIR
#define UART_SEL              P3SEL
#define UART_OUT              P3OUT

// UART UCA1 
#define UART_A1_RX            PIN7
#define UART_A1_TX            PIN6

// UART UCA0 
#define UART_A0_RX            PIN5
#define UART_A0_TX            PIN4

/******************************************************************************
* GPIO Functionality 
******************************************************************************/
// LED
#define LED_DIR                P4DIR
#define LED_SEL                P4SEL
#define LED_OUT                P4OUT
#define PIN_LED                BIT7
#define PIN_LED_1              BIT6       // PLL_LOCK
#define PIN_LED_2              BIT5       // GPS_LOCK

// Accelerometer Interrupt Pins
#define INTTXCL1_DIR           P2DIR
#define INTTXCL2_DIR           P2DIR
#define INTTXCL1_SEL           P2SEL
#define INTTXCL2_SEL           P2SEL
#define INTTXCL1_OUT           P2OUT
#define INTTXCL2_OUT           P2OUT
#define PIN_INTTXCL1           BIT2
#define PIN_INTTXCL2           BIT1

// Encoder Interrupt Pins 
#define ENCODER1_DIR          P2DIR
#define ENCODER1_SEL          P2SEL
#define ENCODER1_OUT          P2OUT
#define PIN_ENCODER1          BIT0

#define ENCODER2_DIR          P1DIR
#define ENCODER2_SEL          P1SEL
#define ENCODER2_OUT          P1OUT
#define PIN_ENCODER2          BIT7

// Pulse Per Second from CDH Watchdog
#define PPS_DIR               P1DIR
#define PPS_SEL               P1SEL
#define PPS_OUT               P1OUT
#define PIN_PPS               BIT5

// UBLOX LEA_6 GPS functionality
#define GPS_FUN_DIR           P1DIR
#define GPS_FUN_SEL           P1SEL
#define GPS_FUN_OUT           P1OUT
#define PIN_GPS_CFGCOM1       BIT4
#define PIN_GPS_EXTINT0       BIT3
#define PIN_GPS_TIMEPULSE     BIT2
#define PIN_GPS_RST           BIT0

// I2C RESET
#define I2C_RST_DIR           P6DIR
#define I2C_RST_SEL           P6SEL
#define I2C_RST_OUT           P6OUT
#define PIN_I2C_RST           BIT4

// Motor Controller 
#define MOTOR_CTRL_DIR        P6DIR
#define MOTOR_CTRL_SEL        P6SEL
#define MOTOR_CTRL_OUT        P6OUT
#define PIN_MOTOR_CTRL        BIT5

/******************************************************************************
*
* Clock configurations (found in init.c)
*   May be slightly inaccurate due to variable nature of DCO
*
******************************************************************************/
#define MCLK_SPD             20000000 // Hz
#define SMCLK_SPD            20000000 // Hz
#define ACLK_SPD             32768    // Hz

#endif 
/* __CONFIG_H */

