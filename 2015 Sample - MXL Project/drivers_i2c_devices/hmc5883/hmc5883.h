/******************************************
 *  ____    ____   ____  ____   _____     *	
 * |_   \  /   _| |_  _||_  _| |_   _|    *
 *   |   \/   |     \ \  / /     | |      *
 *   | |\  /| |      > `' <      | |   _  *
 *  _| |_\/_| |_   _/ /'`\ \_   _| |__/ | *	
 * |_____||_____| |____||____| |________| *	
 *                                        *	
 ******************************************/
 
/*
* adapted by: 	Joshua McCready 
* email:	jmccread@umich.edu
* date:	07/23/2014
*/


#ifndef _HMC5883_H_
#define _HMC5883_H_

#include "I2Cdev.h"


#define HMC5883L_ADDRESS            0x1E // this device only has one address
#define HMC5883L_DEFAULT_ADDRESS    0x1E

#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02
#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAZ_L         0x06
#define HMC5883L_RA_DATAY_H         0x07
#define HMC5883L_RA_DATAY_L         0x08
#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

// Configuration Register A
#define HMC5883L_DO_RATE_SHIFT 	    2
#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06

#define HMC5883L_BIAS_SHIFT	    0
#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_SAMPLE_AVE_SHIFT   5
#define HMC5883L_SA_1               0x00
#define HMC5883L_SA_2               0x01
#define HMC5883L_SA_4               0x02
#define HMC5883L_SA_8               0x03

// Configuration Register B
#define HMC5883L_GAIN_SHIFT	    5
#define HMC5883L_GAIN_1280          0x00
#define HMC5883L_GAIN_1024          0x01
#define HMC5883L_GAIN_768           0x02
#define HMC5883L_GAIN_614           0x03
#define HMC5883L_GAIN_415           0x04
#define HMC5883L_GAIN_361           0x05
#define HMC5883L_GAIN_307           0x06
#define HMC5883L_GAIN_219           0x07

// Mode Register 
#define HMC5883L_MODE_SHIFT         0
#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

// Status Register 
#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0

extern uint16_t hmc5883_gain; 
HMC5883(uint8_t address);
void initialize();

// DATA* registers
uint8_t getHeading(int16_t *x, int16_t *y, int16_t *z); 
int16_t getHeadingX();
int16_t getHeadingY();
int16_t getHeadingZ();

// STATUS register
int getLockStatus();
int getReadyStatus();

// ID_* registers
uint8_t getIDA();
uint8_t getIDB();
uint8_t getIDC();

// Other functionality
HMC5883(uint8_t address);
uint8_t initialize_HMC5883();
int testConnection_HMC5883();
uint8_t SelfTest_HMC5883(int16_t *x_m, int16_t *y_m, int16_t *z_m);
uint8_t Calibrate_HMC5883(float * x_scale, float * y_scale, float * z_scale);
#endif /* _HMC5883L_H_ */
