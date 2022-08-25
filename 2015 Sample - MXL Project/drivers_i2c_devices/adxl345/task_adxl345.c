/**
 * author: 	Joshua McCready 
 * email:	jmccread@umich.edu
 * date:	07/23/2014
 */

/******************************************************************************
*
* task_adxl345.c
*
******************************************************************************/
#include "task_adxl345.h"

void task_adxl345(void)
{   
  int16_t ax, ay, az;
  I2C_SEL |= (PIN_SDA+PIN_SCL);     // Assign I2C pins to usci_b0;
   // Setup I2C according to config.h
  #ifdef I2C_UCB0
   I2C_Init_Master(USCI_B_I2C_CLOCKSOURCE_SMCLK,
                     SMCLK_SPD,
                     USCI_B_I2C_SET_DATA_RATE_100KBPS);
  #endif
  configure_ADXL345(ADXL345_ADDRESS_ALT_LOW);
  while (1) 
  { 
    getAcceleration(&ax, &ay, &az);
    OS_Delay(TASK_ADXL345_DELAY);
  }

}
