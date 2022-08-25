/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\MSP430\\Src\\effs_thin_mmc_drv_ucb0.c,v $
$Author: aek $
$Revision: 3.2 $
$Date: 2012-08-30 10:56:22-07 $

Derived from HCC Embedded's mmc_drv_uca0.h (c)2003 for the MSP430FG461x. Used 
with permission.

EFFS-THIN MMC/SD Card driver for MSP430F261x + CubeSat Kit, using USCI_B0:SPI.

******************************************************************************/
// Does it save? 

/******************************************************************************
In 'spi_init' configure SCLK, MISO, and MOSI to desired pins. Set clock registers
as desired. Set SPI_IE bits as desired.
******************************************************************************/

#include "config.h"
#include "effs_thin_mmc_drv_ucb0.h"
#define ENABLE_VARIABLE_MAX_BAUDRATE  1

#if ENABLE_VARIABLE_MAX_BAUDRATE
unsigned long spi_max_baudrate = 1250000; // in Hz
#endif


/******************************************************************************
****                                                                       ****
**                                                                           **
spi_tx1()
spi_tx512()

Transmit 1 byte.
Transmit 512 bytes.

**                                                                           **
****                                                                       ****
******************************************************************************/
void spi_tx1(unsigned char _data8) {

  SPI_RXBUF;
  SPI_TXBUF = _data8;
  while (SPI_STAT & UCBUSY) {
    ;
  } /* while */
  
} /* spi_tx1() */


void spi_tx512(unsigned char *buf) {

  unsigned short i;


  for (i=0; i<512; i++) {
    SPI_RXBUF;
    SPI_TXBUF = *buf++;
    while (SPI_STAT & UCBUSY) {
      ;
    } /* while */
  } /* for */
  
} /* spi_tx512() */


/******************************************************************************
****                                                                       ****
**                                                                           **
spi_rx1()
spi_rx512()

Receive 1 byte.
Receive 512 bytes.

**                                                                           **
****                                                                       ****
******************************************************************************/
unsigned char spi_rx1(void) {

  spi_tx1(0xff);
  return SPI_RXBUF;
  
} /* spi_rx1() */


void spi_rx512(unsigned char *buf) {

  unsigned short i;
  
  
  for (i=0; i<512; i++) {
    spi_tx1(0xff);
    *buf++ = SPI_RXBUF;
  } /* for */

} /* spi_rx512() */


/******************************************************************************
****                                                                       ****
**                                                                           **
spi_init()

Init SPI peripheral.

**                                                                           **
****                                                                       ****
******************************************************************************/
int spi_init (void) {

  // Configure -CS_SD. Set it HIGH before setting DIR in order to avoid 
  //  spurious changes.
  SPIPORT_CS_SEL &= ~SPICS; 
  SPIPORT_CS     |=  SPICS; 
  SPIPORT_CS_DIR |=  SPICS;

  // Configure SCLK, MISO & MOSI.
  SPI_PORT_SEL   |=  SPISCLK+SPIMISO+SPIMOSI;
  SPI_PORT_DIR   |=  SPISCLK+SPIMOSI;
  SPI_PORT_DIR   &= ~SPIMISO;

  // Configure remaining SPI registers.
  SPI_CTL1        =  UCSSEL1+UCSSEL0 +UCSWRST; 
  SPI_CTL0        =  UCCKPH+UCMSB+UCMST+UCSYNC;
  #ifdef SD_UCA0
    SPI_IE         &= ~(UCA0TXIE+UCA0RXIE);
  #endif 

  #ifdef SD_UCB1
    SPI_IE         &= ~(UCB1TXIE+UCB1RXIE);
  #endif 
  SPI_CTL1       &= ~UCSWRST;
  
  // Done, leave with -CS_SD HIGH.
  spi_cs_hi();
  return 0;

} /* spi_init() */



/******************************************************************************
****                                                                       ****
**                                                                           **
spi_delete()

Delete SPI peripheral.

**                                                                           **
****                                                                       ****
******************************************************************************/
#if 0
int spi_delete(void) {

  SPI_CTL1         =  1;
  SPI_CTL0         =  0;
  SPI_PORT_SEL    &= ~0x0e;  // Deselect MOSI, MISO and CLK on port.
  SPIPORT_CS_DIR  &= ~SPICS;
  SPIPORT_SDS_DIR &= ~SPISDS;
  return 0;

} /* spi_delete() */
#endif

/******************************************************************************
****                                                                       ****
**                                                                           **
spi_set_baudrate()

Set SPI baudrate.

INPUT: br - baudrate

Note: forcing the divide calcs at runtime ensures that the SMCLK and 
CSK_SD_MAX_SPEED symbols can be defined as symbols in Hz without having 
problems with the preprocessor's handling of large numbers.

**                                                                           **
****                                                                       ****
******************************************************************************/
void spi_set_baudrate(unsigned long br) {

  unsigned long div;
  // Either run at 100kHz (startup) or at target-specific maximum SD Card
  //  interface speed.
  if (br != CSK_SD_INIT_SPEED) {
    #if ENABLE_VARIABLE_MAX_BAUDRATE
    br = spi_max_baudrate;
    #else
    br = CSK_SD_MAX_SPEED;
    #endif
  }

  //div =CSK_SD_MAX_SPEED/br ;
  div = 10; // 10 does about 1.9 MHz clk, 8 does about 2.4 MHz
  SPI_CTL1 |=  UCSWRST;
  SPI_BR0   =  div&0xff;
  SPI_BR1   =  div>>8;
  SPI_CTL1 &= ~UCSWRST;
  
} /* spi_set_baudrate() */


/******************************************************************************
****                                                                       ****
**                                                                           **
spi_get_baudrate()

Get SPI baudrate.

Return: baudrate

**                                                                           **
****                                                                       ****
******************************************************************************/
unsigned long spi_get_baudrate(void) {

  unsigned short div = SPI_BR1<<8;


  div += SPI_BR0;
  return SMCLK/div;

} /* spi_get_baudrate() */


/******************************************************************************
****                                                                       ****
**                                                                           **
spi_set_max_baudrate()

Set maximum SPI baudrate. User can change the baud rate at which EFFS-THIN
interacts with the SD Card by calling this before calling f_initvolume().

Note: No bounds checking!

**                                                                           **
****                                                                       ****
******************************************************************************/
#if ENABLE_VARIABLE_MAX_BAUDRATE

void spi_set_max_baudrate(unsigned long br) {

  spi_max_baudrate = br;

} /* spi_set_max_baudrate() */

#endif /* ENABLE_VARIABLE_MAX_BAUDRATE */


/******************************************************************************
****                                                                       ****
**                                                                           **
spi_get_max_baudrate()

Get maximum SPI baudrate.

Return: baudrate

**                                                                           **
****                                                                       ****
******************************************************************************/
#if ENABLE_VARIABLE_MAX_BAUDRATE

unsigned long spi_get_max_baudrate(void) {

  return spi_max_baudrate;

} /* spi_get_max_baudrate() */

#endif /* ENABLE_VARIABLE_MAX_BAUDRATE */
