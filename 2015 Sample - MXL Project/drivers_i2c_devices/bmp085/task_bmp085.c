/******************************************************************************


******************************************************************************/
// Includes
#include "usci_b_i2c.h"
#include "task_bmp085.h"
#include "bmp085.h"
#inlclude "msp430.h"
#include "events.h"
#include "driverlib.h" //
#include "I2C.h"
#include "Config.h" //
#include "salvo.h"
#include "main.h" 

short temperature;
long pressure;

void task_bmp085(void)
{   
  I2C_SEL |= (PIN_SDA+PIN_SCL);     // Assign I2C pins to usci_b0;
  I2C_Init_Master(USCI_B_I2C_CLOCKSOURCE_SMCLK,
                  1000000,
                  USCI_B_I2C_SET_DATA_RATE_100KBPS);
  bmp085Calibration();
 
  while (1) 
  { 
    // Temperature measure is in 0.1 degrees C
    bmp085ReadUT();
    // Wait at least 4.5ms, assuming OS_timer at 1 ms ticks
    OS_Delay(10);
   // Adding Temperature into global data container, change for specific applications 
    data.bmp_temp = bmp085GetTemperature();
  
    // Pressure measured in Pa
    bmp085ReadUP();
    // Wait for conversion, delay time dependent on OSS, assuming  1 ms ticks
    // bitshifted 0 corresponds to over sampling setting 
    OS_Delay(2 + (3<<0));

    // Adding pressure into global data container, change for specific applications 
    data.pressure = bmp085GetPressure();

    OS_Delay(100);
  }

}