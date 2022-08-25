/* --COPYRIGHT--,BSD
 * Copyright (c) 2013, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#ifndef __HW_MEMMAP__
#define __HW_MEMMAP__

//*****************************************************************************
//
// Include device specific header file
//
//*****************************************************************************

#include <msp430.h>


/************************************************************
* USCI Ax
************************************************************/

#define OFS_UCAxCTLW0          (0x0000)       /* USCI Ax Control Word Register 0 */
#define OFS_UCAxCTLW0_L        OFS_UCAxCTLW0
#define OFS_UCAxCTLW0_H        OFS_UCAxCTLW0+1
#define OFS_UCAxCTL0           (0x0001)
#define OFS_UCAxCTL1           (0x0000)
#define UCAxCTL1               UCAxCTLW0_L    /* USCI Ax Control Register 1 */
#define UCAxCTL0               UCAxCTLW0_H    /* USCI Ax Control Register 0 */
#define OFS_UCAxBRW            (0x0006)       /* USCI Ax Baud Word Rate 0 */
#define OFS_UCAxBRW_L          OFS_UCAxBRW
#define OFS_UCAxBRW_H          OFS_UCAxBRW+1
#define OFS_UCAxBR0            (0x0006)
#define OFS_UCAxBR1            (0x0007)
#define UCAxBR0                UCAxBRW_L      /* USCI Ax Baud Rate 0 */
#define UCAxBR1                UCAxBRW_H      /* USCI Ax Baud Rate 1 */
#define OFS_UCAxMCTL           (0x0008)       /* USCI Ax Modulation Control */
#define OFS_UCAxSTAT           (0x000A)       /* USCI Ax Status Register */
#define OFS_UCAxRXBUF          (0x000C)       /* USCI Ax Receive Buffer */
#define OFS_UCAxTXBUF          (0x000E)       /* USCI Ax Transmit Buffer */
#define OFS_UCAxABCTL          (0x0010)       /* USCI Ax LIN Control */
#define OFS_UCAxIRCTL          (0x0012)       /* USCI Ax IrDA Transmit Control */
#define OFS_UCAxIRCTL_L        OFS_UCAxIRCTL
#define OFS_UCAxIRCTL_H        OFS_UCAxIRCTL+1
#define OFS_UCAxIRTCTL         (0x0012)
#define OFS_UCAxIRRCTL         (0x0013)
#define UCAxIRTCTL             UCAxIRCTL_L    /* USCI Ax IrDA Transmit Control */
#define UCAxIRRCTL             UCAxIRCTL_H    /* USCI Ax IrDA Receive Control */
#define OFS_UCAxICTL           (0x001C)       /* USCI Ax Interrupt Enable Register */
#define OFS_UCAxICTL_L         OFS_UCAxICTL
#define OFS_UCAxICTL_H         OFS_UCAxICTL+1
#define OFS_UCAxIE             (0x001C)
#define OFS_UCAxIFG            (0x001D)
#define UCAxIE                 UCAxICTL_L     /* USCI Ax Interrupt Enable Register */
#define UCAxIFG                UCAxICTL_H     /* USCI Ax Interrupt Flags Register */
#define OFS_UCAxIV             (0x001E)       /* USCI Ax Interrupt Vector Register */

#define OFS_UCAxCTLW0__SPI     (0x0000)
#define OFS_UCAxCTLW0__SPI_L   OFS_UCAxCTLW0__SPI
#define OFS_UCAxCTLW0__SPI_H   OFS_UCAxCTLW0__SPI+1
#define OFS_UCAxCTL0__SPI      (0x0001)
#define OFS_UCAxCTL1__SPI      (0x0000)
#define OFS_UCAxBRW__SPI       (0x0006)
#define OFS_UCAxBRW__SPI_L     OFS_UCAxBRW__SPI
#define OFS_UCAxBRW__SPI_H     OFS_UCAxBRW__SPI+1
#define OFS_UCAxBR0__SPI       (0x0006)
#define OFS_UCAxBR1__SPI       (0x0007)
#define OFS_UCAxMCTL__SPI      (0x0008)
#define OFS_UCAxSTAT__SPI      (0x000A)
#define OFS_UCAxRXBUF__SPI     (0x000C)
#define OFS_UCAxTXBUF__SPI     (0x000E)
#define OFS_UCAxICTL__SPI      (0x001C)
#define OFS_UCAxICTL__SPI_L    OFS_UCAxICTL__SPI
#define OFS_UCAxICTL__SPI_H    OFS_UCAxICTL__SPI+1
#define OFS_UCAxIE__SPI        (0x001C)
#define OFS_UCAxIFG__SPI       (0x001D)
#define OFS_UCAxIV__SPI        (0x001E)

/************************************************************
* USCI Bx
************************************************************/

#define OFS_UCBxCTLW0__SPI     (0x0000)
#define OFS_UCBxCTLW0__SPI_L   OFS_UCBxCTLW0__SPI
#define OFS_UCBxCTLW0__SPI_H   OFS_UCBxCTLW0__SPI+1
#define OFS_UCBxCTL0__SPI      (0x0001)
#define OFS_UCBxCTL1__SPI      (0x0000)
#define OFS_UCBxBRW__SPI       (0x0006)
#define OFS_UCBxBRW__SPI_L     OFS_UCBxBRW__SPI
#define OFS_UCBxBRW__SPI_H     OFS_UCBxBRW__SPI+1
#define OFS_UCBxBR0__SPI       (0x0006)
#define OFS_UCBxBR1__SPI       (0x0007)
#define OFS_UCBxSTAT__SPI      (0x000A)
#define OFS_UCBxRXBUF__SPI     (0x000C)
#define OFS_UCBxTXBUF__SPI     (0x000E)
#define OFS_UCBxICTL__SPI      (0x001C)
#define OFS_UCBxICTL__SPI_L    OFS_UCBxICTL__SPI
#define OFS_UCBxICTL__SPI_H    OFS_UCBxICTL__SPI+1
#define OFS_UCBxIE__SPI        (0x001C)
#define OFS_UCBxIFG__SPI       (0x001D)
#define OFS_UCBxIV__SPI        (0x001E)

#define OFS_UCBxCTLW0          (0x0000)       /* USCI Bx Control Word Register 0 */
#define OFS_UCBxCTLW0_L        OFS_UCBxCTLW0
#define OFS_UCBxCTLW0_H        OFS_UCBxCTLW0+1
#define OFS_UCBxCTL0           (0x0001)
#define OFS_UCBxCTL1           (0x0000)
#define UCBxCTL1               UCBxCTLW0_L    /* USCI Bx Control Register 1 */
#define UCBxCTL0               UCBxCTLW0_H    /* USCI Bx Control Register 0 */
#define OFS_UCBxBRW            (0x0006)       /* USCI Bx Baud Word Rate 0 */
#define OFS_UCBxBRW_L          OFS_UCBxBRW
#define OFS_UCBxBRW_H          OFS_UCBxBRW+1
#define OFS_UCBxBR0            (0x0006)
#define OFS_UCBxBR1            (0x0007)
#define UCBxBR0                UCBxBRW_L      /* USCI Bx Baud Rate 0 */
#define UCBxBR1                UCBxBRW_H      /* USCI Bx Baud Rate 1 */
#define OFS_UCBxSTAT           (0x000A)       /* USCI Bx Status Register */
#define OFS_UCBxRXBUF          (0x000C)       /* USCI Bx Receive Buffer */
#define OFS_UCBxTXBUF          (0x000E)       /* USCI Bx Transmit Buffer */
#define OFS_UCBxI2COA          (0x0010)       /* USCI Bx I2C Own Address */
#define OFS_UCBxI2COA_L        OFS_UCBxI2COA
#define OFS_UCBxI2COA_H        OFS_UCBxI2COA+1
#define OFS_UCBxI2CSA          (0x0012)       /* USCI Bx I2C Slave Address */
#define OFS_UCBxI2CSA_L        OFS_UCBxI2CSA
#define OFS_UCBxI2CSA_H        OFS_UCBxI2CSA+1
#define OFS_UCBxICTL           (0x001C)       /* USCI Bx Interrupt Enable Register */
#define OFS_UCBxICTL_L         OFS_UCBxICTL
#define OFS_UCBxICTL_H         OFS_UCBxICTL+1
#define OFS_UCBxIE             (0x001C)
#define OFS_UCBxIFG            (0x001D)
#define UCBxIE                 UCBxICTL_L     /* USCI Bx Interrupt Enable Register */
#define UCBxIFG                UCBxICTL_H     /* USCI Bx Interrupt Flags Register */
#define OFS_UCBxIV             (0x001E)       /* USCI Bx Interrupt Vector Register */


// UCAxCTL0 UART-Mode Control Bits
#define UCPEN                  (0x80)         /* Async. Mode: Parity enable */
#define UCPAR                  (0x40)         /* Async. Mode: Parity     0:odd / 1:even */
#define UCMSB                  (0x20)         /* Async. Mode: MSB first  0:LSB / 1:MSB */
#define UC7BIT                 (0x10)         /* Async. Mode: Data Bits  0:8-bits / 1:7-bits */
#define UCSPB                  (0x08)         /* Async. Mode: Stop Bits  0:one / 1: two */
#define UCMODE1                (0x04)         /* Async. Mode: USCI Mode 1 */
#define UCMODE0                (0x02)         /* Async. Mode: USCI Mode 0 */
#define UCSYNC                 (0x01)         /* Sync-Mode  0:UART-Mode / 1:SPI-Mode */

// UCxxCTL0 SPI-Mode Control Bits
#define UCCKPH                 (0x80)         /* Sync. Mode: Clock Phase */
#define UCCKPL                 (0x40)         /* Sync. Mode: Clock Polarity */
#define UCMST                  (0x08)         /* Sync. Mode: Master Select */

// UCBxCTL0 I2C-Mode Control Bits
#define UCA10                  (0x80)         /* 10-bit Address Mode */
#define UCSLA10                (0x40)         /* 10-bit Slave Address Mode */
#define UCMM                   (0x20)         /* Multi-Master Environment */
//#define res               (0x10)    /* reserved */
#define UCMODE_0               (0x00)         /* Sync. Mode: USCI Mode: 0 */
#define UCMODE_1               (0x02)         /* Sync. Mode: USCI Mode: 1 */
#define UCMODE_2               (0x04)         /* Sync. Mode: USCI Mode: 2 */
#define UCMODE_3               (0x06)         /* Sync. Mode: USCI Mode: 3 */

// UCAxCTL1 UART-Mode Control Bits
#define UCSSEL1                (0x80)         /* USCI 0 Clock Source Select 1 */
#define UCSSEL0                (0x40)         /* USCI 0 Clock Source Select 0 */
#define UCRXEIE                (0x20)         /* RX Error interrupt enable */
#define UCBRKIE                (0x10)         /* Break interrupt enable */
#define UCDORM                 (0x08)         /* Dormant (Sleep) Mode */
#define UCTXADDR               (0x04)         /* Send next Data as Address */
#define UCTXBRK                (0x02)         /* Send next Data as Break */
#define UCSWRST                (0x01)         /* USCI Software Reset */

// UCxxCTL1 SPI-Mode Control Bits
//#define res               (0x20)    /* reserved */
//#define res               (0x10)    /* reserved */
//#define res               (0x08)    /* reserved */
//#define res               (0x04)    /* reserved */
//#define res               (0x02)    /* reserved */

// UCBxCTL1 I2C-Mode Control Bits
//#define res               (0x20)    /* reserved */
#define UCTR                   (0x10)         /* Transmit/Receive Select/Flag */
#define UCTXNACK               (0x08)         /* Transmit NACK */
#define UCTXSTP                (0x04)         /* Transmit STOP */
#define UCTXSTT                (0x02)         /* Transmit START */
#define UCSSEL_0               (0x00)         /* USCI 0 Clock Source: 0 */
#define UCSSEL_1               (0x40)         /* USCI 0 Clock Source: 1 */
#define UCSSEL_2               (0x80)         /* USCI 0 Clock Source: 2 */
#define UCSSEL_3               (0xC0)         /* USCI 0 Clock Source: 3 */
#define UCSSEL__UCLK           (0x00)         /* USCI 0 Clock Source: UCLK */
#define UCSSEL__ACLK           (0x40)         /* USCI 0 Clock Source: ACLK */
#define UCSSEL__SMCLK          (0x80)         /* USCI 0 Clock Source: SMCLK */

/* UCAxMCTL Control Bits */
#define UCBRF3                 (0x80)         /* USCI First Stage Modulation Select 3 */
#define UCBRF2                 (0x40)         /* USCI First Stage Modulation Select 2 */
#define UCBRF1                 (0x20)         /* USCI First Stage Modulation Select 1 */
#define UCBRF0                 (0x10)         /* USCI First Stage Modulation Select 0 */
#define UCBRS2                 (0x08)         /* USCI Second Stage Modulation Select 2 */
#define UCBRS1                 (0x04)         /* USCI Second Stage Modulation Select 1 */
#define UCBRS0                 (0x02)         /* USCI Second Stage Modulation Select 0 */
#define UCOS16                 (0x01)         /* USCI 16-times Oversampling enable */

#define UCBRF_0                (0x00)         /* USCI First Stage Modulation: 0 */
#define UCBRF_1                (0x10)         /* USCI First Stage Modulation: 1 */
#define UCBRF_2                (0x20)         /* USCI First Stage Modulation: 2 */
#define UCBRF_3                (0x30)         /* USCI First Stage Modulation: 3 */
#define UCBRF_4                (0x40)         /* USCI First Stage Modulation: 4 */
#define UCBRF_5                (0x50)         /* USCI First Stage Modulation: 5 */
#define UCBRF_6                (0x60)         /* USCI First Stage Modulation: 6 */
#define UCBRF_7                (0x70)         /* USCI First Stage Modulation: 7 */
#define UCBRF_8                (0x80)         /* USCI First Stage Modulation: 8 */
#define UCBRF_9                (0x90)         /* USCI First Stage Modulation: 9 */
#define UCBRF_10               (0xA0)         /* USCI First Stage Modulation: A */
#define UCBRF_11               (0xB0)         /* USCI First Stage Modulation: B */
#define UCBRF_12               (0xC0)         /* USCI First Stage Modulation: C */
#define UCBRF_13               (0xD0)         /* USCI First Stage Modulation: D */
#define UCBRF_14               (0xE0)         /* USCI First Stage Modulation: E */
#define UCBRF_15               (0xF0)         /* USCI First Stage Modulation: F */

#define UCBRS_0                (0x00)         /* USCI Second Stage Modulation: 0 */
#define UCBRS_1                (0x02)         /* USCI Second Stage Modulation: 1 */
#define UCBRS_2                (0x04)         /* USCI Second Stage Modulation: 2 */
#define UCBRS_3                (0x06)         /* USCI Second Stage Modulation: 3 */
#define UCBRS_4                (0x08)         /* USCI Second Stage Modulation: 4 */
#define UCBRS_5                (0x0A)         /* USCI Second Stage Modulation: 5 */
#define UCBRS_6                (0x0C)         /* USCI Second Stage Modulation: 6 */
#define UCBRS_7                (0x0E)         /* USCI Second Stage Modulation: 7 */

/* UCAxSTAT Control Bits */
#define UCLISTEN               (0x80)         /* USCI Listen mode */
#define UCFE                   (0x40)         /* USCI Frame Error Flag */
#define UCOE                   (0x20)         /* USCI Overrun Error Flag */
#define UCPE                   (0x10)         /* USCI Parity Error Flag */
#define UCBRK                  (0x08)         /* USCI Break received */
#define UCRXERR                (0x04)         /* USCI RX Error Flag */
#define UCADDR                 (0x02)         /* USCI Address received Flag */
#define UCBUSY                 (0x01)         /* USCI Busy Flag */
#define UCIDLE                 (0x02)         /* USCI Idle line detected Flag */

/* UCBxSTAT Control Bits */
#define UCSCLLOW               (0x40)         /* SCL low */
#define UCGC                   (0x20)         /* General Call address received Flag */
#define UCBBUSY                (0x10)         /* Bus Busy Flag */

/* UCAxIRTCTL Control Bits */
#define UCIRTXPL5              (0x80)         /* IRDA Transmit Pulse Length 5 */
#define UCIRTXPL4              (0x40)         /* IRDA Transmit Pulse Length 4 */
#define UCIRTXPL3              (0x20)         /* IRDA Transmit Pulse Length 3 */
#define UCIRTXPL2              (0x10)         /* IRDA Transmit Pulse Length 2 */
#define UCIRTXPL1              (0x08)         /* IRDA Transmit Pulse Length 1 */
#define UCIRTXPL0              (0x04)         /* IRDA Transmit Pulse Length 0 */
#define UCIRTXCLK              (0x02)         /* IRDA Transmit Pulse Clock Select */
#define UCIREN                 (0x01)         /* IRDA Encoder/Decoder enable */

/* UCAxIRRCTL Control Bits */
#define UCIRRXFL5              (0x80)         /* IRDA Receive Filter Length 5 */
#define UCIRRXFL4              (0x40)         /* IRDA Receive Filter Length 4 */
#define UCIRRXFL3              (0x20)         /* IRDA Receive Filter Length 3 */
#define UCIRRXFL2              (0x10)         /* IRDA Receive Filter Length 2 */
#define UCIRRXFL1              (0x08)         /* IRDA Receive Filter Length 1 */
#define UCIRRXFL0              (0x04)         /* IRDA Receive Filter Length 0 */
#define UCIRRXPL               (0x02)         /* IRDA Receive Input Polarity */
#define UCIRRXFE               (0x01)         /* IRDA Receive Filter enable */

/* UCAxABCTL Control Bits */
//#define res               (0x80)    /* reserved */
//#define res               (0x40)    /* reserved */
#define UCDELIM1               (0x20)         /* Break Sync Delimiter 1 */
#define UCDELIM0               (0x10)         /* Break Sync Delimiter 0 */
#define UCSTOE                 (0x08)         /* Sync-Field Timeout error */
#define UCBTOE                 (0x04)         /* Break Timeout error */
//#define res               (0x02)    /* reserved */
#define UCABDEN                (0x01)         /* Auto Baud Rate detect enable */

/* UCBxI2COA Control Bits */
#define UCGCEN                 (0x8000)       /* I2C General Call enable */
#define UCOA9                  (0x0200)       /* I2C Own Address 9 */
#define UCOA8                  (0x0100)       /* I2C Own Address 8 */
#define UCOA7                  (0x0080)       /* I2C Own Address 7 */
#define UCOA6                  (0x0040)       /* I2C Own Address 6 */
#define UCOA5                  (0x0020)       /* I2C Own Address 5 */
#define UCOA4                  (0x0010)       /* I2C Own Address 4 */
#define UCOA3                  (0x0008)       /* I2C Own Address 3 */
#define UCOA2                  (0x0004)       /* I2C Own Address 2 */
#define UCOA1                  (0x0002)       /* I2C Own Address 1 */
#define UCOA0                  (0x0001)       /* I2C Own Address 0 */

/* UCBxI2COA Control Bits */
#define UCOA7_L                (0x0080)       /* I2C Own Address 7 */
#define UCOA6_L                (0x0040)       /* I2C Own Address 6 */
#define UCOA5_L                (0x0020)       /* I2C Own Address 5 */
#define UCOA4_L                (0x0010)       /* I2C Own Address 4 */
#define UCOA3_L                (0x0008)       /* I2C Own Address 3 */
#define UCOA2_L                (0x0004)       /* I2C Own Address 2 */
#define UCOA1_L                (0x0002)       /* I2C Own Address 1 */
#define UCOA0_L                (0x0001)       /* I2C Own Address 0 */

/* UCBxI2COA Control Bits */
#define UCGCEN_H               (0x0080)       /* I2C General Call enable */
#define UCOA9_H                (0x0002)       /* I2C Own Address 9 */
#define UCOA8_H                (0x0001)       /* I2C Own Address 8 */

/* UCBxI2CSA Control Bits */
#define UCSA9                  (0x0200)       /* I2C Slave Address 9 */
#define UCSA8                  (0x0100)       /* I2C Slave Address 8 */
#define UCSA7                  (0x0080)       /* I2C Slave Address 7 */
#define UCSA6                  (0x0040)       /* I2C Slave Address 6 */
#define UCSA5                  (0x0020)       /* I2C Slave Address 5 */
#define UCSA4                  (0x0010)       /* I2C Slave Address 4 */
#define UCSA3                  (0x0008)       /* I2C Slave Address 3 */
#define UCSA2                  (0x0004)       /* I2C Slave Address 2 */
#define UCSA1                  (0x0002)       /* I2C Slave Address 1 */
#define UCSA0                  (0x0001)       /* I2C Slave Address 0 */

/* UCBxI2CSA Control Bits */
#define UCSA7_L                (0x0080)       /* I2C Slave Address 7 */
#define UCSA6_L                (0x0040)       /* I2C Slave Address 6 */
#define UCSA5_L                (0x0020)       /* I2C Slave Address 5 */
#define UCSA4_L                (0x0010)       /* I2C Slave Address 4 */
#define UCSA3_L                (0x0008)       /* I2C Slave Address 3 */
#define UCSA2_L                (0x0004)       /* I2C Slave Address 2 */
#define UCSA1_L                (0x0002)       /* I2C Slave Address 1 */
#define UCSA0_L                (0x0001)       /* I2C Slave Address 0 */

/* UCBxI2CSA Control Bits */
#define UCSA9_H                (0x0002)       /* I2C Slave Address 9 */
#define UCSA8_H                (0x0001)       /* I2C Slave Address 8 */

/* UCAxIE Control Bits */
#define UCTXIE                 (0x0002)       /* USCI Transmit Interrupt Enable */
#define UCRXIE                 (0x0001)       /* USCI Receive Interrupt Enable */

/* UCAxIE Control Bits */
#define UCTXIE_L               (0x0002)       /* USCI Transmit Interrupt Enable */
#define UCRXIE_L               (0x0001)       /* USCI Receive Interrupt Enable */

/* UCBxIE Control Bits */
#define UCNACKIE               (0x0020)       /* NACK Condition interrupt enable */
#define UCALIE                 (0x0010)       /* Arbitration Lost interrupt enable */
#define UCSTPIE                (0x0008)       /* STOP Condition interrupt enable */
#define UCSTTIE                (0x0004)       /* START Condition interrupt enable */
#define UCTXIE                 (0x0002)       /* USCI Transmit Interrupt Enable */
#define UCRXIE                 (0x0001)       /* USCI Receive Interrupt Enable */

/* UCBxIE Control Bits */
#define UCNACKIE_L             (0x0020)       /* NACK Condition interrupt enable */
#define UCALIE_L               (0x0010)       /* Arbitration Lost interrupt enable */
#define UCSTPIE_L              (0x0008)       /* STOP Condition interrupt enable */
#define UCSTTIE_L              (0x0004)       /* START Condition interrupt enable */
#define UCTXIE_L               (0x0002)       /* USCI Transmit Interrupt Enable */
#define UCRXIE_L               (0x0001)       /* USCI Receive Interrupt Enable */

/* UCAxIFG Control Bits */
#define UCTXIFG                (0x0002)       /* USCI Transmit Interrupt Flag */
#define UCRXIFG                (0x0001)       /* USCI Receive Interrupt Flag */

/* UCAxIFG Control Bits */
#define UCTXIFG_L              (0x0002)       /* USCI Transmit Interrupt Flag */
#define UCRXIFG_L              (0x0001)       /* USCI Receive Interrupt Flag */

/* UCBxIFG Control Bits */
#define UCNACKIFG              (0x0020)       /* NAK Condition interrupt Flag */
#define UCALIFG                (0x0010)       /* Arbitration Lost interrupt Flag */
#define UCSTPIFG               (0x0008)       /* STOP Condition interrupt Flag */
#define UCSTTIFG               (0x0004)       /* START Condition interrupt Flag */
#define UCTXIFG                (0x0002)       /* USCI Transmit Interrupt Flag */
#define UCRXIFG                (0x0001)       /* USCI Receive Interrupt Flag */

/* UCBxIFG Control Bits */
#define UCNACKIFG_L            (0x0020)       /* NAK Condition interrupt Flag */
#define UCALIFG_L              (0x0010)       /* Arbitration Lost interrupt Flag */
#define UCSTPIFG_L             (0x0008)       /* STOP Condition interrupt Flag */
#define UCSTTIFG_L             (0x0004)       /* START Condition interrupt Flag */
#define UCTXIFG_L              (0x0002)       /* USCI Transmit Interrupt Flag */
#define UCRXIFG_L              (0x0001)       /* USCI Receive Interrupt Flag */

/* USCI Definitions */
#define USCI_NONE              (0x0000)       /* No Interrupt pending */
#define USCI_UCRXIFG           (0x0002)       /* USCI UCRXIFG */
#define USCI_UCTXIFG           (0x0004)       /* USCI UCTXIFG */
#define USCI_I2C_UCALIFG       (0x0002)       /* USCI I2C Mode: UCALIFG */
#define USCI_I2C_UCNACKIFG     (0x0004)       /* USCI I2C Mode: UCNACKIFG */
#define USCI_I2C_UCSTTIFG      (0x0006)       /* USCI I2C Mode: UCSTTIFG*/
#define USCI_I2C_UCSTPIFG      (0x0008)       /* USCI I2C Mode: UCSTPIFG*/
#define USCI_I2C_UCRXIFG       (0x000A)       /* USCI I2C Mode: UCRXIFG */
#define USCI_I2C_UCTXIFG       (0x000C)       /* USCI I2C Mode: UCTXIFG */


//*****************************************************************************
//
// The following are defines for the base address of the peripherals.
//
//*****************************************************************************
/*
#ifdef __MSP430_HAS_ADC10_A__
                                                                        #define ADC10_A_BASE    __MSP430_BASEADDRESS_ADC10_A__
#endif
#ifdef __MSP430_HAS_ADC10_B__
                                                                        #define ADC10_B_BASE    __MSP430_BASEADDRESS_ADC10_B__
#endif
#ifdef __MSP430_HAS_ADC12_B__
                                                                        #define ADC12_B_BASE    __MSP430_BASEADDRESS_ADC12_B__
#endif
#ifdef __MSP430_HAS_ADC12_PLUS__
                                                                        #define ADC12_A_BASE    __MSP430_BASEADDRESS_ADC12_PLUS__
#endif
#ifdef __MSP430_HAS_AES256__
                                                                        #define AES256_BASE             __MSP430_BASEADDRESS_AES256__
#endif
#ifdef __MSP430_HAS_AES__
                                                                        #define AES_BASE                __MSP430_BASEADDRESS_AES__
#endif
#ifdef __MSP430_HAS_AUX_SUPPLY__
                                                                        #define AUX_SUPPLY_BASE __MSP430_BASEADDRESS_AUX_SUPPLY__
#endif
#ifdef __MSP430_HAS_BACKUP_RAM__
                                                                        #define BAK_RAM_BASE    __MSP430_BASEADDRESS_BACKUP_RAM__
#endif
#ifdef __MSP430_HAS_BATTERY_CHARGER__
                                                                        #define BAK_BATT_BASE   __MSP430_BASEADDRESS_BATTERY_CHARGER__
#endif
#ifdef __MSP430_HAS_CAP_SENSE_IO_0__
                                                                        #define CAP_TOUCH_0_BASE        __MSP430_BASEADDRESS_CAP_SENSE_IO_0__
#endif
#ifdef __MSP430_HAS_CAP_SENSE_IO_1__
                                                                        #define CAP_TOUCH_1_BASE        __MSP430_BASEADDRESS_CAP_SENSE_IO_1__
#endif
#ifdef __MSP430_HAS_COMPB__
                                                                        #define COMP_B_BASE             __MSP430_BASEADDRESS_COMPB__
#endif
#ifdef __MSP430_HAS_COMPD__
                                                                        #define COMP_D_BASE             __MSP430_BASEADDRESS_COMPD__
#endif
#ifdef __MSP430_HAS_COMP_E__
                                                                        #define COMP_E_BASE             __MSP430_BASEADDRESS_COMP_E__
#endif
#ifdef __MSP430_HAS_COMP_E__
                                                                        #define __MSP430_BASEADDRESS_COMPE__            __MSP430_BASEADDRESS_COMP_E__
#endif
#ifdef __MSP430_HAS_CRC__
                                                                        #define CRC_BASE                __MSP430_BASEADDRESS_CRC__
#endif
#ifdef __MSP430_HAS_CS__
                                                                        #define CS_BASE                 __MSP430_BASEADDRESS_CS__
#endif
#ifdef __MSP430_HAS_DAC12_2__
                                                                        #define DAC12_A_BASE    __MSP430_BASEADDRESS_DAC12_2__
#endif
#ifdef __MSP430_HAS_DMAX_3__
                                                                        #define DMA_BASE                __MSP430_BASEADDRESS_DMAX_3__
#endif
#ifdef __MSP430_HAS_DMAX_6__
                                                                        #define DMA_BASE                __MSP430_BASEADDRESS_DMAX_6__
#endif
#ifdef __MSP430_HAS_EUSCI_A0__
                                                                        #define EUSCI_A0_BASE           __MSP430_BASEADDRESS_EUSCI_A0__
#endif
#ifdef __MSP430_HAS_EUSCI_A1__
                                                                        #define EUSCI_A1_BASE           __MSP430_BASEADDRESS_EUSCI_A1__
#endif
#ifdef __MSP430_HAS_EUSCI_A2__
                                                                        #define EUSCI_A2_BASE           __MSP430_BASEADDRESS_EUSCI_A2__
#endif
#ifdef __MSP430_HAS_EUSCI_A3__
                                                                        #define EUSCI_A3_BASE           __MSP430_BASEADDRESS_EUSCI_A3__
#endif
#ifdef __MSP430_HAS_EUSCI_B0__
                                                                        #define EUSCI_B0_BASE           __MSP430_BASEADDRESS_EUSCI_B0__
#endif
#ifdef __MSP430_HAS_EUSCI_B1__
                                                                        #define EUSCI_B1_BASE           __MSP430_BASEADDRESS_EUSCI_B1__
#endif
#ifdef __MSP430_HAS_FLASH__
                                                                        #define FLASH_BASE              __MSP430_BASEADDRESS_FLASH__
#endif
#ifdef __MSP430_HAS_FRAM_FR5XX__
                                                                        #define FRAM_BASE               __MSP430_BASEADDRESS_FRAM_FR5XX__
#endif
#ifdef __MSP430_HAS_FRAM__
                                                                        #define FRAM_BASE               __MSP430_BASEADDRESS_FRAM__
#endif
#ifdef __MSP430_HAS_LCD_B__
                                                                        #define LCD_B_BASE              __MSP430_BASEADDRESS_LCD_B__
#endif
#ifdef __MSP430_HAS_LCD_C__
                                                                        #define LCD_C_BASE              __MSP430_BASEADDRESS_LCD_C__
#endif
#ifdef __MSP430_HAS_MPU_A__
                                                                        #define MPU_BASE                __MSP430_BASEADDRESS_MPU_A__
#endif
#ifdef __MSP430_HAS_MPU__
                                                                        #define MPU_BASE        __MSP430_BASEADDRESS_MPU__
#endif
#ifdef __MSP430_HAS_MPY32__
                                                                        #define MPY32_BASE              __MSP430_BASEADDRESS_MPY32__
#endif
#ifdef __MSP430_HAS_PMM_FR5xx__
                                                                        #define PMM_BASE        __MSP430_BASEADDRESS_PMM_FR5xx__
#endif
#ifdef __MSP430_HAS_PMM_FRAM__
                                                                        #define PMM_BASE        __MSP430_BASEADDRESS_PMM_FRAM__
#endif
#ifdef __MSP430_HAS_PMM__
                                                                        #define PMM_BASE        __MSP430_BASEADDRESS_PMM__
#endif
#ifdef __MSP430_HAS_PORT10_R__
                                                                        #define P10_BASE                __MSP430_BASEADDRESS_PORT10_R__
#endif
#ifdef __MSP430_HAS_PORT11_R__
                                                                        #define P11_BASE                __MSP430_BASEADDRESS_PORT11_R__
#endif
#ifdef __MSP430_HAS_PORT1_MAPPING__
                                                                        #define P1MAP_BASE              __MSP430_BASEADDRESS_PORT1_MAPPING__
#endif
#ifdef __MSP430_HAS_PORT1_R__
                                                                        #define P1_BASE         __MSP430_BASEADDRESS_PORT1_R__
#endif
#ifdef __MSP430_HAS_PORT2_MAPPING__
                                                                        #define P2MAP_BASE      __MSP430_BASEADDRESS_PORT2_MAPPING__
#endif
#ifdef __MSP430_HAS_PORT2_R__
                                                                        #define P2_BASE         __MSP430_BASEADDRESS_PORT2_R__
#endif
#ifdef __MSP430_HAS_PORT3_MAPPING__
                                                                        #define P3MAP_BASE              __MSP430_BASEADDRESS_PORT3_MAPPING__
#endif
#ifdef __MSP430_HAS_PORT3_R__
                                                                        #define P3_BASE         __MSP430_BASEADDRESS_PORT3_R__
#endif
#ifdef __MSP430_HAS_PORT4_MAPPING__
                                                                        #define P4MAP_BASE              __MSP430_BASEADDRESS_PORT4_MAPPING__
#endif
#ifdef __MSP430_HAS_PORT4_R__
                                                                        #define P4_BASE         __MSP430_BASEADDRESS_PORT4_R__
#endif
#ifdef __MSP430_HAS_PORT5_R__
                                                                        #define P5_BASE         __MSP430_BASEADDRESS_PORT5_R__
#endif
#ifdef __MSP430_HAS_PORT6_R__
                                                                        #define P6_BASE         __MSP430_BASEADDRESS_PORT6_R__
#endif
#ifdef __MSP430_HAS_PORT7_R__
                                                                        #define P7_BASE         __MSP430_BASEADDRESS_PORT7_R__
#endif
#ifdef __MSP430_HAS_PORT8_R__
                                                                        #define P8_BASE         __MSP430_BASEADDRESS_PORT8_R__
#endif
#ifdef __MSP430_HAS_PORT9_R__
                                                                        #define P9_BASE         __MSP430_BASEADDRESS_PORT9_R__
#endif
#ifdef __MSP430_HAS_PORTA_R__
                                                                        #define PA_BASE         __MSP430_BASEADDRESS_PORTA_R__
#endif
#ifdef __MSP430_HAS_PORTB_R__
                                                                        #define PB_BASE         __MSP430_BASEADDRESS_PORTB_R__
#endif
#ifdef __MSP430_HAS_PORTC_R__
                                                                        #define PC_BASE         __MSP430_BASEADDRESS_PORTC_R__
#endif
#ifdef __MSP430_HAS_PORTD_R__
                                                                        #define PD_BASE         __MSP430_BASEADDRESS_PORTD_R__
#endif
#ifdef __MSP430_HAS_PORTE_R__
                                                                        #define PE_BASE         __MSP430_BASEADDRESS_PORTE_R__
#endif
#ifdef __MSP430_HAS_PORTF_R__
                                                                        #define PF_BASE         __MSP430_BASEADDRESS_PORTF_R__
#endif
#ifdef __MSP430_HAS_PORTJ_R__
                                                                        #define PJ_BASE         __MSP430_BASEADDRESS_PORTJ_R__
#endif
#ifdef __MSP430_HAS_PORT_MAPPING__
                                                                        #define PMAP_CTRL_BASE          __MSP430_BASEADDRESS_PORT_MAPPING__
#endif
#ifdef __MSP430_HAS_PU__
                                                                        #define LDOPWR_BASE             __MSP430_BASEADDRESS_PU__
#endif
#ifdef __MSP430_HAS_RC__
                                                                        #define RAM_BASE                __MSP430_BASEADDRESS_RC__
#endif
#ifdef __MSP430_HAS_REF_A__
                                                                        #define REF_A_BASE              __MSP430_BASEADDRESS_REF_A__
#endif
#ifdef __MSP430_HAS_REF__
                                                                        #define REF_BASE                __MSP430_BASEADDRESS_REF__
#endif
#ifdef __MSP430_HAS_RTC_B__
                                                                        #define RTC_B_BASE              __MSP430_BASEADDRESS_RTC_B__
#endif
#ifdef __MSP430_HAS_RTC_C__
                                                                        #define RTC_C_BASE              __MSP430_BASEADDRESS_RTC_C__
#endif
#ifdef __MSP430_HAS_RTC_D__
                                                                        #define RTC_D_BASE              __MSP430_BASEADDRESS_RTC_D__
#endif
#ifdef __MSP430_HAS_RTC__
                                                                        #define RTC_A_BASE              __MSP430_BASEADDRESS_RTC__
#endif
#ifdef __MSP430_HAS_SD24_B__
                                                                        #define SD24_BASE               __MSP430_BASEADDRESS_SD24_B__
#endif
#ifdef __MSP430_HAS_SFR__
                                                                    #define SFR_BASE            __MSP430_BASEADDRESS_SFR__
#endif
#ifdef __MSP430_HAS_SYS__
                                                                        #define SYS_BASE                __MSP430_BASEADDRESS_SYS__
#endif
#ifdef __MSP430_HAS_T0A3__
                                                                        #define TIMER_A0_BASE           __MSP430_BASEADDRESS_T0A3__
#endif
#ifdef __MSP430_HAS_T0A5__
                                                                        #define TIMER_A0_BASE           __MSP430_BASEADDRESS_T0A5__
#endif
#ifdef __MSP430_HAS_T0B3__
                                                                        #define TIMER_B0_BASE           __MSP430_BASEADDRESS_T0B3__
#endif
#ifdef __MSP430_HAS_T0B7__
                                                                        #define TIMER_B0_BASE           __MSP430_BASEADDRESS_T0B7__
#endif
#ifdef __MSP430_HAS_T0D3__
                                                                        #define TIMER_D0_BASE           __MSP430_BASEADDRESS_T0D3__
#endif
#ifdef __MSP430_HAS_T1A2__
                                                                        #define TIMER_A1_BASE           __MSP430_BASEADDRESS_T1A2__
#endif
#ifdef __MSP430_HAS_T1A3__
                                                                        #define TIMER_A1_BASE           __MSP430_BASEADDRESS_T1A3__
#endif
#ifdef __MSP430_HAS_T1B3__
                                                                        #define TIMER_B1_BASE           __MSP430_BASEADDRESS_T1B3__
#endif
#ifdef __MSP430_HAS_T1D3__
                                                                        #define TIMER_D1_BASE           __MSP430_BASEADDRESS_T1D3__
#endif
#ifdef __MSP430_HAS_T2A2__
                                                                        #define TIMER_A2_BASE           __MSP430_BASEADDRESS_T2A2__
#endif
#ifdef __MSP430_HAS_T2A3__
                                                                        #define TIMER_A2_BASE           __MSP430_BASEADDRESS_T2A3__
#endif
#ifdef __MSP430_HAS_T2B3__
                                                                        #define TIMER_B2_BASE           __MSP430_BASEADDRESS_T2B3__
#endif
#ifdef __MSP430_HAS_T3A2__
                                                                        #define TIMER_A3_BASE           __MSP430_BASEADDRESS_T3A2__
#endif
#ifdef __MSP430_HAS_TEV0__
                                                                        #define TEC0_BASE               __MSP430_BASEADDRESS_TEV0__
#endif
#ifdef __MSP430_HAS_TEV1__
                                                                        #define TEC1_BASE               __MSP430_BASEADDRESS_TEV1__
#endif
#ifdef __MSP430_HAS_UCS__
                                                                        #define UCS_BASE                __MSP430_BASEADDRESS_UCS__
#endif
#ifdef __MSP430_HAS_USB__
                                                                        #define USB_BASE                __MSP430_BASEADDRESS_USB__
#endif
#ifdef __MSP430_HAS_USCI_A0__
                                                                        #define USCI_A0_BASE            __MSP430_BASEADDRESS_USCI_A0__
#endif
#ifdef __MSP430_HAS_USCI_A1__
                                                                        #define USCI_A1_BASE            __MSP430_BASEADDRESS_USCI_A1__
#endif
#ifdef __MSP430_HAS_USCI_A2__
                                                                        #define USCI_A2_BASE            __MSP430_BASEADDRESS_USCI_A2__
#endif
#ifdef __MSP430_HAS_USCI_A3__
                                                                        #define USCI_A3_BASE            __MSP430_BASEADDRESS_USCI_A3__
#endif
#ifdef __MSP430_HAS_USCI_B0__
                                                                        #define USCI_B0_BASE            __MSP430_BASEADDRESS_USCI_B0__
#endif
#ifdef __MSP430_HAS_USCI_B1__
                                                                        #define USCI_B1_BASE            __MSP430_BASEADDRESS_USCI_B1__
#endif
#ifdef __MSP430_HAS_USCI_B2__
                                                                        #define USCI_B2_BASE            __MSP430_BASEADDRESS_USCI_B2__
#endif
#ifdef __MSP430_HAS_USCI_B3__
                                                                        #define USCI_B3_BASE            __MSP430_BASEADDRESS_USCI_B3__
#endif
#ifdef __MSP430_HAS_WDT_A__
                                                                        #define WDT_A_BASE                      __MSP430_BASEADDRESS_WDT_A__
#endif
*/
#endif // #ifndef __HW_MEMMAP__
