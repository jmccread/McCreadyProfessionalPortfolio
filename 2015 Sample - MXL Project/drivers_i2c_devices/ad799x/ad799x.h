
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
#ifndef __ad799x_h
#define __ad799x_h
#include "stdint.h"
/********************************************************************************
*
* Address select brief: 
*	the address select pin (AS) is tied to VDD; 
*	however, AS can also be tied to AGND or left floating, allowing 
*	the user to select up to five AD7993/AD7994 devices on the 
* 	same serial bus. Up to five AD7993/AD7994 devices can be connected to the 
* 	serial bus without isolation in standard or fast mode. In high speed mode 
*   only up to three devices can be conected 
*
 *******************************************************************************/

// Define address space for various AD7993/7994 part numbers,
#define AD7993_0GND_ADDR                                0x21
#define AD7993_0VDD_ADDR                                0x22
#define AD7993_1GND_ADDR                                0x23
#define AD7993_1VDD_ADDR                                0x24
#define AD7994_0GND_ADDR                                0x21
#define AD7994_0VDD_ADDR                                0x22
#define AD7994_1GND_ADDR                                0x23
#define AD7994_1VDD_ADDR                                0x24

// Internal Address Pointer Register Addresses P0 - P3
#define CONV_RES 					0x00			//Conversion Result Register (Read)
#define ALR_STAT					0x01			//Alert Status Register (Read/Write)
#define CFG_REG						0x02			//Configuration Register (Read/Write) 
#define CYC_TIMER					0x03			//Cycle Timer Register (Read/Write) 
#define DAT_L_CH1					0x04			//DATALOW Reg CH1 (Read/Write)
#define DAT_H_CH1					0x05			//DATAHIGH Reg CH1 (Read/Write)
#define HYST                                            0x06			//Hysteresis Reg CH1 (Read/Write) 
#define DAT_L_CH1					0x04			//DATALOW Reg CH1 (Read/Write)
#define DAT_H_CH1					0x05			//DATAHIGH Reg CH1 (Read/Write)
#define HYST_CH1					0x06			//Hysteresis Reg CH1 (Read/Write) 
#define DAT_L_CH2					0x07			//DATALOW Reg CH2 (Read/Write)
#define DAT_H_CH2					0x08			//DATAHIGH Reg CH2 (Read/Write)
#define HYST_CH2					0x09			//Hysteresis Reg CH2 (Read/Write) 
#define DAT_L_CH3					0x0A			//DATALOW Reg CH3 (Read/Write)
#define DAT_H_CH3					0x0B			//DATAHIGH Reg CH3 (Read/Write)
#define HYST_CH3					0x0C			//Hysteresis Reg CH3 (Read/Write) 
#define DAT_L_CH4					0x0D			//DATALOW Reg CH4 (Read/Write)
#define DAT_H_CH4					0x0E			//DATAHIGH Reg CH4 (Read/Write)
#define HYST_CH4					0x0F			//Hysteresis Reg CH4 (Read/Write)

/********************************************************************************
*
* Configuration Register  brief: 
*	The macros below are used to configure the CFG_REG, please refer to page 19
*	of the datasheet for more information
*
 *******************************************************************************/
// Configuration register macros C1-C4 and D4-D7
#define CH1                                             0x10
#define CH2                                             0x20
#define CH3                                             0x40
#define CH4                                             0x80

// D0-D3
#define FLTR						0x08
#define ALR_EN						0x04
#define BUSYALR                                         0x02
#define BUSYALR_POL					0x01 

// Configuration (measure and input correct value)
#define AD799x_Vref                                     2.5

/********************************************************************************
*
* Function Prototypes 
*
 *******************************************************************************/
uint8_t command_mode_read(uint8_t channels, uint8_t address, uint16_t channel_int[4]);




#endif /* __main_h */