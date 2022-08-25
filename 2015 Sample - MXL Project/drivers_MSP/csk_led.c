/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\MSP430\\Src\\csk_led.c,v $
$Author: aek $
$Revision: 3.0 $
$Date: 2009-11-01 22:54:41-08 $

******************************************************************************/

/******************************************************************************
Change functions to correspond
******************************************************************************/
#include "csk_led.h"
#include "csk_hw.h"
#include "config.h"

/******************************************************************************
****                                                                       ****
**                                                                           **
csk_led_status_open()

Open the status LED for use.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_led_status_open(uint8_t pin) {

  LED_SEL &= ~pin;
  LED_DIR |= pin;
  
} /* csk_led_status_open() */


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_led_status_close()

Close the status LED.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_led_status_close(uint8_t pin) {

  // Configure P1.0 as an input.
  LED_SEL &= ~pin;
  LED_DIR &= ~pin;
  
} /* csk_led_status_close() */


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_led_status_on()

Turn the status LED ON.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_led_status_on(uint8_t pin) {

  // Take status LED on HIGH.        
  LED_OUT |=  pin;

  
} /* csk_led_status_on() */


/******************************************************************************
****                                                                       ****
**                                                                           **
csk_led_status_off()

Turn the status LED OFF.

**                                                                           **
****                                                                       ****
******************************************************************************/
void csk_led_status_off(uint8_t pin) {

  // Take status LED on P1.0 LOW.            
  LED_OUT &= ~pin;
  
} /* csk_led_status_off() */
