/******************************************************************************


******************************************************************************/
// Includes
#ifndef __bmp085_h
#define __bmp085_h
#include "stdint.h"


//Other defintions  
#define USCI_SMCLK_DESIRED_FREQUENCY_IN_HZ    1000000   
#define BMP085_ADDR                           0x77

void bmp085Calibration();
void bmp085ReadUT(void);
void bmp085ReadUP(void);
int bmp085ReadInt(uint8_t address);
short bmp085GetTemperature(void);
long bmp085GetPressure(void);
#endif 
