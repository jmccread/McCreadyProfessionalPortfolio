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
*
* task_ad799x.c 
* A basic task for sampling the four channel ad7994 adc 
*
******************************************************************************/

// Includes
#include "task_ad799x.h"
#include "ad799x.h"
#inlclude "msp430.h"
#include "events.h"
#include "driverlib.h" //
#include "I2C.h"
#include "Config.h" //
#include "salvo.h"
#include "main.h" 


void task_ad799x(void)
{   
  uint16_t measure[4];
  I2C_SEL |= (PIN_SDA+PIN_SCL);     // Assign I2C pins to usci_b0;
 // Setup I2C according to config.h
 #ifdef I2C_UCB0
 I2C_Init_Master(USCI_B_I2C_CLOCKSOURCE_SMCLK,
                 SMCLK_SPD,
                 USCI_B_I2C_SET_DATA_RATE_100KBPS);
 #endif
 
  while (1) 
  { 
    command_mode_read(CH1|CH2, AD7994_1VDD_ADDR,  &measure);
    OS_Delay(TASK_AD799x_DELAY);
  }

}