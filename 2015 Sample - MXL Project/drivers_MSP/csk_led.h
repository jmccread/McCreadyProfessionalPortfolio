/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\MSP430\\Inc\\csk_led.h,v $
$Author: aek $
$Revision: 3.0 $
$Date: 2009-11-01 22:54:18-08 $

******************************************************************************/
#ifndef __csk_led_h
#define __csk_led_h
#include "stdint.h"

extern void csk_led_status_close(uint8_t pin);
extern void csk_led_status_off(uint8_t pin);
extern void csk_led_status_on(uint8_t pin);
extern void csk_led_status_open(uint8_t pin);
extern void csk_led_toggle(uint8_t pin);

#endif /* __csk_led_h */
