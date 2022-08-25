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
* author: Joshua McCready 
* email:	jmccread@umich.edu
* date:	07/14/2014
*/

/******************************************************************************
* Includes 
******************************************************************************/
#include "task_l3g4200d.h"
#include "l3g4200d.h"
#inlclude "msp430.h"
#include "events.h"
#include "driverlib.h" //
#include "I2C.h"
#include "Config.h" //
#include "salvo.h"
#include "main.h" 

/******************************************************************************
* Globals  
******************************************************************************/
uint8_t error_task_l3g4200d=0x00; 

/******************************************************************************
*
* task_l3g4200d()
*
******************************************************************************/
void task_l3g4200d(void)
{   
 float l3g4200d_temp;
 I2C_SEL |= (PIN_SDA+PIN_SCL);   
 
 // Setup I2C according to config.h
 #ifdef I2C_UCB0
 I2C_Init_Master(USCI_B_I2C_CLOCKSOURCE_SMCLK,
                 SMCLK_SPD,
                 USCI_B_I2C_SET_DATA_RATE_100KBPS);
 #endif

 while (1) 
 {  
   error_task_l3g4200d= measureL3G4200D(l3g4200d_1SAD_ADDR, &l3g4200d_temp,
    &data.gyro_x, &data.gyro_y, &data.gyro_z);
   OS_Delay(TASK_L3G4200D_DELAY);
 }
}