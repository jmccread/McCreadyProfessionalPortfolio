#include "HMC5883.h"
uint8_t devAddr_HMC5883; 
uint8_t buffer_HMC5883[6];
uint8_t mode_HMC5883;
uint16_t hmc5883_gain; 

/** Address assigned 
 * @param address I2C address
 * @see HMC5883L_DEFAULT_ADDRESS
 * @see HMC5883L_ADDRESS
 */

void HMC5883L(uint8_t address) {
    devAddr_HMC5883 = address;
}


/** Self test operation for calibration
 * This will configure the magnetometer to collect measurements after establishing
 * its own predictable magnetic field which can be used to calibrate further 
 * measurements.
 * See datasheet page 19.
 */
 uint8_t SelfTest_HMC5883(int16_t *x_m, int16_t *y_m, int16_t *z_m) { 
   uint8_t err =0x00; 
   HMC5883L(HMC5883L_ADDRESS);
 
   /*To implement this self test, the least significant bits 
  (MS1 and MS0) of configuration register A are changed from 00 to 01 */
   err = writeByte(devAddr_HMC5883, HMC5883L_RA_CONFIG_A, 0x71);

   /* Return Configuration register B to factory default to more easily predict measurement 
   * values returned from self test. */ 
   err = writeByte(devAddr_HMC5883, HMC5883L_RA_CONFIG_B, 0xA0); 

   /* By placing the mode register into continous-measurement mode (0x00), two data acquisition 
   cycles will be made on each magnetic vector*/ 
   err = writeByte(devAddr_HMC5883, HMC5883L_RA_MODE, 0x00); 

    delay_ms(150);
  
   getHeading(x_m, y_m, z_m);
   if (!err) {
     if (!((*x_m > 244) & (*x_m < 575)))
       err = 0x01;
     if (!((*y_m > 244) & (*y_m < 575)))
       err = err+0x02;
     if (!((*z_m > 244) & (*z_m < 575)))
       err = err+0x03;
     }
   return err; 
}

/** 
*
*/ 
uint8_t Calibrate_HMC5883(float * x_scale, float * y_scale, float * z_scale) {
   
   // Measured values from self test
   int16_t x_m, y_m, z_m, err; 

   // Expected values to compare test data to (from data sheet)
   // (Experiment at room temp in 205 lab yields 460, 430m and 421) 
   float x_e = 452;
   float y_e = 452;
   float z_e = 421;

   // Perform self test
   err = SelfTest_HMC5883(&x_m, &y_m, &z_m); 

   if (!err) {
   // Calculate scale factors
   *x_scale = (float)x_m/x_e;
   *y_scale = (float)y_m/y_e;
   *z_scale = (float)z_m/z_e;
   }
   // If there was an error don't spit out bad things 
   else
   {
    *x_scale = 1;
    *y_scale = 1;
    *z_scale = 1;
   }
   return err; 
}

/** Power on and prepare for general usage.
 * This will prepare the magnetometer with default settings, ready for single-
 * use mode_HMC5883 (very low power requirements). Default settings include 8-sample
 * averaging, 15 Hz data output rate, normal measurement bias, a,d 1090 gain (in
 * terms of LSB/Gauss). Be sure to adjust any settings you need specifically
 * after initialization, especially the gain settings if you happen to be seeing
 * a lot of -4096 values (see the datasheet for mor information).
 */
uint8_t initialize_HMC5883(void) {
    uint8_t err; 

    // write CONFIG_A register
    HMC5883L(HMC5883L_ADDRESS);
    err= writeByte(devAddr_HMC5883, HMC5883L_RA_CONFIG_A, (HMC5883L_RATE_75 << HMC5883L_DO_RATE_SHIFT) | (HMC5883L_SA_8 << HMC5883L_SAMPLE_AVE_SHIFT)
     | 0x00);

    // write CONFIG_B register
    err= writeByte(devAddr_HMC5883, HMC5883L_RA_CONFIG_B, 0x00 | (HMC5883L_GAIN_1024 << HMC5883L_GAIN_SHIFT));
    hmc5883_gain = 1024;

    // write MODE register
    err= writeByte(devAddr_HMC5883, HMC5883L_RA_MODE, (0X00 |
				HMC5883L_MODE_CONTINUOUS));
		     mode_HMC5883 = HMC5883L_MODE_CONTINUOUS;
    delay_ms(6);

    return err; 
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
int testConnection_HMC5883(void) {
    if (readBytes(devAddr_HMC5883, HMC5883L_RA_ID_A, 3, buffer_HMC5883, 0) == 3) {
        return (buffer_HMC5883[0] == 'H' && buffer_HMC5883[1] == '4' && buffer_HMC5883[2] == '3');
    }
    return 0;
}



// DATA* registers

/** Get 3-axis heading measurements.
 * In the event the ADC reading overflows or underflows for the given channel,
 * or if there is a math overflow during the bias measurement, this data
 * register will contain the value -4096. This register value will clear when
 * after the next valid measurement is made. Note that this method automatically
 * clears the appropriate bit in the MODE register if Single mode_HMC5883 is active.
 * @param x 16-bit signed integer container for X-axis heading
 * @param y 16-bit signed integer container for Y-axis heading
 * @param z 16-bit signed integer container for Z-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
uint8_t getHeading(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t eu; 
    eu= readBytes(devAddr_HMC5883, HMC5883L_RA_DATAX_H, 6, buffer_HMC5883, 0);
    if (~eu)
    {
      if (mode_HMC5883 == HMC5883L_MODE_SINGLE)
       {
        eu= writeByte(devAddr_HMC5883, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << ( HMC5883L_MODE_SHIFT));
        } 
      *x = (((int16_t)buffer_HMC5883[0]) << 8) | buffer_HMC5883[1];
      *y = (((int16_t)buffer_HMC5883[4]) << 8) | buffer_HMC5883[5];
      *z = (((int16_t)buffer_HMC5883[2]) << 8) | buffer_HMC5883[3];
    }
    else
    {
      *x = 0;
      *y = 0;
      *z = 0; 
    }
    return eu; 
}
/** Get X-axis heading measurement.
 * @return 16-bit signed integer with X-axis heading
 * @see HMC5883L_RA_DATAX_H
 */
int16_t getHeadingX() {
    // each axis read requires that ALL axis registers be read, even if only
    // one is used; this was not done ineffiently in the code by accident
    readBytes(devAddr_HMC5883, HMC5883L_RA_DATAX_H, 6, buffer_HMC5883, 0);
    if (mode_HMC5883 == HMC5883L_MODE_SINGLE) writeByte(devAddr_HMC5883, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << ( HMC5883L_MODE_SHIFT));
    return (((int16_t)buffer_HMC5883[0]) << 8) | buffer_HMC5883[1];
}
/** Get Y-axis heading measurement.
 * @return 16-bit signed integer with Y-axis heading
 * @see HMC5883L_RA_DATAY_H
 */
int16_t getHeadingY() {
    // each axis read requires that ALL axis registers be read, even if only
    // one is used; this was not done ineffiently in the code by accident
    readBytes(devAddr_HMC5883, HMC5883L_RA_DATAX_H, 6, buffer_HMC5883, 0);
    if (mode_HMC5883 == HMC5883L_MODE_SINGLE) writeByte(devAddr_HMC5883, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << ( HMC5883L_MODE_SHIFT));
    return (((int16_t)buffer_HMC5883[4]) << 8) | buffer_HMC5883[5];
}
/** Get Z-axis heading measurement.
 * @return 16-bit signed integer with Z-axis heading
 * @see HMC5883L_RA_DATAZ_H
 */
int16_t getHeadingZ() {
    // each axis read requires that ALL axis registers be read, even if only
    // one is used; this was not done ineffiently in the code by accident
    readBytes(devAddr_HMC5883, HMC5883L_RA_DATAX_H, 6, buffer_HMC5883, 0);
    if (mode_HMC5883 == HMC5883L_MODE_SINGLE) writeByte(devAddr_HMC5883, HMC5883L_RA_MODE, HMC5883L_MODE_SINGLE << HMC5883L_MODE_SHIFT);
    return (((int16_t)buffer_HMC5883[2]) << 8) | buffer_HMC5883[3];
}

// STATUS register

/** Get data output register lock status.
 * This bit is set when this some but not all for of the six data output
 * registers have been read. When this bit is set, the six data output registers
 * are locked and any new data will not be placed in these register until one of
 * three conditions are met: one, all six bytes have been read or the mode_HMC5883
 * changed, two, the mode_HMC5883 is changed, or three, the measurement configuration is
 * changed.
 * @return Data output register lock status
 * @see HMC5883L_RA_STATUS
 * @see HMC5883L_STATUS_LOCK_BIT
 */
int getLockStatus() {
    readBit(devAddr_HMC5883, HMC5883L_RA_STATUS, HMC5883L_STATUS_LOCK_BIT, buffer_HMC5883, 0);
    return buffer_HMC5883[0];
}
/** Get data ready status.
 * This bit is set when data is written to all six data registers, and cleared
 * when the device initiates a write to the data output registers and after one
 * or more of the data output registers are written to. When RDY bit is clear it
 * shall remain cleared for 250 us. DRDY pin can be used as an alternative to
 * the status register for monitoring the device for measurement data.
 * @return Data ready status
 * @see HMC5883L_RA_STATUS
 * @see HMC5883L_STATUS_READY_BIT
 */
int getReadyStatus() {
    readBit(devAddr_HMC5883, HMC5883L_RA_STATUS, HMC5883L_STATUS_READY_BIT, buffer_HMC5883, 0);
    return buffer_HMC5883[0];
}

// ID_* registers

/** Get identification byte A
 * @return ID_A byte (should be 01001000, ASCII value 'H')
 */
uint8_t getIDA() {
    readByte(devAddr_HMC5883, HMC5883L_RA_ID_A, buffer_HMC5883, 0);
    return buffer_HMC5883[0];
}
/** Get identification byte B
 * @return ID_A byte (should be 00110100, ASCII value '4')
 */
uint8_t getIDB() {
    readByte(devAddr_HMC5883, HMC5883L_RA_ID_B, buffer_HMC5883, 0);
    return buffer_HMC5883[0];
}
/** Get identification byte C
 * @return ID_A byte (should be 00110011, ASCII value '3')
 */
uint8_t getIDC() {
    readByte(devAddr_HMC5883, HMC5883L_RA_ID_C, buffer_HMC5883, 0);
    return buffer_HMC5883[0];
}


