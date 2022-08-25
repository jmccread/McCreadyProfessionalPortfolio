/**
 * author: 	Joshua McCready 
 * email:	jmccread@umich.edu
 * date:	07/28/2014
 */

/******************************************************************************
*
* task_hmc5883.c
*
******************************************************************************/
#include "task_hmc5883.h"

void task_hmc5883(void)
{   
  // Scale factors for each axis determined by calibration procedure 
  static float scale_x, scale_y, scale_z; 
  // measured data magnetometer data
  int16_t x_m, y_m, z_m; 
  // adjusted data containers after scaling
  int16_t x_a, y_a, z_a;

  // Floating point measurements in Gauss calculated using gain configuration 
  float x_gauss, y_gauss, z_gauss; 

  // Counter to recalibration measurements to avoid sensor drift due to temp
  uint32_t calibration_count = 0; 
  uint16_t calibration_f = 5000;

   // Setup I2C according to config.h
   #ifdef I2C_UCB0
   I2C_Init_Master(USCI_B_I2C_CLOCKSOURCE_SMCLK,
                   SMCLK_SPD,
                   USCI_B_I2C_SET_DATA_RATE_100KBPS);
   #endif

  Calibrate_HMC5883(&scale_x, &scale_y, &scale_z);
  initialize();
  while (1) 
  { 
    OS_Delay(TASK_HMC5883_DELAY);
    // Collect measurements 
    getHeading(&x_m, &y_m, &z_m);
    x_a = ((float)x_m*scale_x);
    y_a = ((float)y_m*scale_y);
    z_a = ((float)z_m*scale_z);
    
    x_gauss = (float)x_a/hmc5883_gain;
    y_gauss = (float)y_a/hmc5883_gain;
    z_gauss = (float)z_a/hmc5883_gain;

    calibration_count++; 
    
    if (
    (calibration_count%calibration_f)==0) {
      calibration_count = 0; 
      Calibrate_HMC5883(&scale_x, &scale_y, &scale_z);
      initialize_HMC5883();
      }
  }
}


