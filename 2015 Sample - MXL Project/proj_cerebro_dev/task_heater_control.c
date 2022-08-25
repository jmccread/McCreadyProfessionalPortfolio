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
 * author: 	Andrew Wagenmaker 
 * email:	ajwagen@umich.edu
 * date:	04/10/2015
 */

/******************************************************************************
*
* task_heater_control()
*
******************************************************************************/

/******************************************************************************
* Includes 
******************************************************************************/
#include "task_heater_control.h"
/******************************************************************************
* Globals 
******************************************************************************/
//int on = 0;


void task_heater_control(void) 
{
  /******************************************************************************
  * Setup
  ******************************************************************************/
  //declarations of variables needed
  float median = 0;
  int greater_cnt = 0;
  int less_cnt = 0;
  int i, j;
  static int on = 0;

  //set heater GPIO pin to output state
  //csk_led_status_on(PIN_LED_2);
  ENCODER1_SEL &= ~PIN_ENCODER1;
  ENCODER1_DIR |= PIN_ENCODER1;

  while (1) {

    //run every 30 seconds as defined in init.c
    OS_WaitBinSem(TEMP_CHECK_P, OSNO_TIMEOUT);

    //find the median temperature value sampled from the photodiode boards
    //this will help eliminate errors caused by erroneous measurements
    /*
    for (i = 0; i < 5; ++i)
    {
        greater_cnt = 0;
        less_cnt = 0;

        for (j = 0; j < 5; ++j)
        {
            if (data.adc_temp[i] < data.adc_temp[j])  
                ++greater_cnt;
            else if (data.adc_temp[i] > data.adc_temp[j])
                ++less_cnt;
        }

        if (greater_cnt == 2 && less_cnt == 2)
        {
           median = data.adc_temp[i];
           break;
        }
    }*/
   median = data.bmp_temp/10; 

    if (on == 0 && median < 0.0)
    {
        //if the heater is not already on, and the temperature drops below 0, turn heater on
        on = 1;
        ENCODER1_OUT |= PIN_ENCODER1;
    } else if (on == 1 && median > 5.0) {
       //if the heater is on and the temperature climbs above 5, turn the heater off. Setting the on and off
       //temperatures differently will eliminate the heater oscillating between on and off when around 0 degrees.
       on = 0;
       ENCODER1_OUT &= ~PIN_ENCODER1;
    }

    data.heater_on = on;

    OS_Yield();
  }

}



void task_SHADO(void) 
{
  /******************************************************************************
  * Setup
  ******************************************************************************/
  //set heater GPIO pin to output state
  uint8_t do_once = 1; 
  float elap_t_start; 
  ENCODER2_SEL &= ~PIN_ENCODER2;
  ENCODER2_DIR |= PIN_ENCODER2;
  ENCODER2_OUT &= ~PIN_ENCODER2;
  while (1) {
    // Only wait for the binary semaphor to be triggered once 
    if(do_once) 
    { 
      OS_WaitBinSem(ALT_THRESH_P, OSNO_TIMEOUT);
      do_once = 0; 
      elap_t_start = data.HF_t_elap; 
      ENCODER2_OUT |= PIN_ENCODER2;
    }
    
    if ((elap_t_start+BURN_DURATION) < data.HF_t_elap)
    {
      ENCODER2_OUT &= ~PIN_ENCODER2;
    }


    OS_Yield();
  }

}