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
*
* task_measure_low_f()
*
******************************************************************************/

/******************************************************************************
* Includes 
******************************************************************************/
#include "task_measure_low_f.h"

/******************************************************************************
* Globals 
******************************************************************************/

void task_develop(void) {

  /******************************************************************************
  * Setup
  ******************************************************************************/
   uint16_t channels[4];
   // Setup I2C according to config.h
   #ifdef I2C_UCB0
   I2C_Init_Master(USCI_B_I2C_CLOCKSOURCE_SMCLK,
                   SMCLK_SPD,
                   USCI_B_I2C_SET_DATA_RATE_400KBPS);
   #endif
  
  while(1) 
    {
    I2C_Init_Master(USCI_B_I2C_CLOCKSOURCE_SMCLK,
                   SMCLK_SPD,
                   USCI_B_I2C_SET_DATA_RATE_400KBPS);
    command_mode_read(CH1|CH2|CH3|CH4, PERIPH_ADC, channels);

     OS_Delay(100);
    }
 }

