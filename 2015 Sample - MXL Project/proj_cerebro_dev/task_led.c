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
* task_led()
* Simple task that flashes the LED (when present).
*
******************************************************************************/

/******************************************************************************
* Includes 
******************************************************************************/
#include "config.h"
#include "init.h"
#include "task_led.h"
#include "main.h"
#include "csk_led.h"
#include "salvo.h"




void task_led(void) {
  csk_led_status_open(PIN_LED);  
 
  while (1) { 
    // Flash LED -- on time is 1/10 the off time.
    csk_led_status_off(PIN_LED);
    OS_Delay(TASK_LED_DELAY);
    csk_led_status_on(PIN_LED);
    OS_Delay(TASK_LED_DELAY);    
  } /* while */
} /* task_led() */


