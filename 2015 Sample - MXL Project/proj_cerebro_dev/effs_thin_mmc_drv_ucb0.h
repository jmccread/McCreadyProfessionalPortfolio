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

/************************************************************ 

/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\MSP430\\Inc\\effs_thin_mmc_drv_ucb0.h,v $
$Author: aek $
$Revision: 3.2 $
$Date: 2009-11-02 01:52:20-08 $

Derived from HCC Embedded's mmc_drv_uca0.c (c)2003 for the MSP430FG461x. Used 
with permission.

EFFS-THIN MMC/SD Card driver for MSP430F261x + CubeSat Kit, using USCI_B0:SPI.

******************************************************************************/

/*****************************************************************************
* effs_thin_mmc_drv_ucb0.h
* Note: Really should be changed from this to something else.
******************************************************************************/
#include <config.h>
#ifndef __effs_thin_mmc_drv_ucb0_h
#define __effs_thin_mmc_drv_ucb0_h

#ifdef __cplusplus
extern "C" 
#endif

// For MSP430F2618
#include  <msp430x26x.h>
#include  <main.h>

/******************************************************************************
*
* Definitions 
*
******************************************************************************/
// We run USCI_Xx:SPI from SMCLK
#define SMCLK                SMCLK_SPD

#define CSK_SD_INIT_SPEED    1000000     // Hz
#define CSK_SD_MAX_SPEED     25000000     // Hz

// Pin definitions                     
#define SPICS               CS_SD
#define SPIMOSI             SD_MOSI
#define SPIMISO             SD_MISO
#define SPISCLK             SD_CLK

// Chip select on PX.x.
// Card Detect (CD) and Write Protect (WP) not supported.
#define SPIPORT_CS_DIR      CS_DIR  
#define SPIPORT_CS_SEL      CS_SEL
#define SPIPORT_CS          CS_OUT 
#define SPI_PORT_SEL        SD_SEL                
#define SPI_PORT_DIR        SD_DIR  

#ifdef SD_UCA0
  // Register definitions -- USCI_A0:SPI
  #define SPI_CTL0            UCA0CTL0
  #define SPI_CTL1            UCA0CTL1
  #define SPI_BR0             UCA0BR0
  #define SPI_BR1             UCA0BR1
  #define SPI_MCTL            UCA0MCTL
  #define SPI_STAT            UCA0STAT
  #define SPI_RXBUF           UCA0RXBUF
  #define SPI_TXBUF           UCA0TXBUF
#endif

#ifdef SD_UCB1
  // Register definitions -- USCI_B0:SPI
  #define SPI_CTL0            UCB1CTL0
  #define SPI_CTL1            UCB1CTL1
  #define SPI_BR0             UCB1BR0
  #define SPI_BR1             UCB1BR1
  #define SPI_MCTL            UCB1MCTL
  #define SPI_STAT            UCB1STAT
  #define SPI_RXBUF           UCB1RXBUF
  #define SPI_TXBUF           UCB1TXBUF
#endif

#define SPI_IE              UC1IE                      
#define SPI_IFG             UC1IFG  
#define SPI_IFG_BIT         ((1<<0)|(1<<1))  
         
/******************************************************************************
*
* Function Prototypes and Macros  
*
******************************************************************************/
// MMC driver function declarations.
int spi_init(void);    
int spi_delete(void);                
void spi_set_baudrate(unsigned long);  
unsigned long spi_get_baudrate(void);  
void spi_tx1(unsigned char);
void spi_tx512(unsigned char *);
unsigned char spi_rx1(void);  
void spi_rx512(unsigned char *);

#define spi_cs_lo()         SPIPORT_CS &= ~SPICS
#define spi_cs_hi()         SPIPORT_CS |=  SPICS
#define get_cd()            1
#define get_wp()            0

#ifdef __cplusplus
}
#endif
#endif /* __effs_thin_mmc_drv_ucb0_h */
