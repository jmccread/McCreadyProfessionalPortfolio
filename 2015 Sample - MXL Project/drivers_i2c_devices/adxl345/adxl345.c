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
 * date:	07/01/2014
 */
 


#include "adxl345.h"
#include "task_adxl345.h"
 uint8_t devAddr_ADXL345;
 uint8_t buffer_ADXL345[6];


void ADXL345(uint8_t address) {
    devAddr_ADXL345 = address;
}

/** Power on and prepare for general usage.
 * This will activate the accelerometer, so be sure to adjust the power settings
 * after you call this method if you want it to enter standby mode, or another
 * less demanding mode of operation.
 */
void initialize_ADXL345() {
    uint8_t e;
    e = writeByte(devAddr_ADXL345, ADXL345_RA_POWER_CTL, 0); // reset all power settings
    setAutoSleepEnabled(1);
    setMeasureEnabled(1);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return 1 if connection is valid, false otherwise
 */
int testConnection_ADXL345(void) {
    return getDeviceID_ADXL345() == 0xE5;
}

// DEVID register

/** Get Device ID.
 * The DEVID register holds a fixed device ID code of 0xE5 (345 octal).
 * @return Device ID (should be 0xE5, 229 dec, 345 oct)
 * @see ADXL345_RA_DEVID
 */
uint8_t getDeviceID_ADXL345() {
    readByte(devAddr_ADXL345, ADXL345_RA_DEVID, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}

// THRESH_TAP register

/** Get tap threshold.
 * The THRESH_TAP register is eight bits and holds the threshold value for tap
 * interrupts. The data format is unsigned, therefore, the magnitude of the tap
 * event is compared with the value in THRESH_TAP for normal tap detection. The
 * scale factor is 62.5 mg/LSB (that is, 0xFF = 16 g). A value of 0 may result
 * in undesirable behavior if single tap/double tap interrupts are enabled.
 * @return Tap threshold (scaled at 62.5 mg/LSB)
 * @see ADXL345_RA_THRESH_TAP
 */
uint8_t getTapThreshold() {
    readByte(devAddr_ADXL345, ADXL345_RA_THRESH_TAP, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set tap threshold.
  * @param threshold Tap magnitude threshold (scaled at 62.5 mg/LSB)
  * @see ADXL345_RA_THRESH_TAP
  * @see getTapThreshold()
  */
void AsetTapThreshold(uint8_t threshold) {
    writeByte(devAddr_ADXL345, ADXL345_RA_THRESH_TAP, threshold);
}

// OFS* registers

/** Get axis offsets.
 * The OFSX, OFSY, and OFSZ registers are each eight bits and offer user-set
 * offset adjustments in twos complement format with a scale factor of 15.6
 * mg/LSB (that is, 0x7F = 2 g). The value stored in the offset registers is
 * automatically added to the acceleration data, and the resulting value is
 * stored in the output data registers. For additional information regarding
 * offset calibration and the use of the offset registers, refer to the Offset
 * Calibration section of the datasheet.
 * @param x X axis offset container
 * @param y Y axis offset container
 * @param z Z axis offset container
 * @see ADXL345_RA_OFSX
 * @see ADXL345_RA_OFSY
 * @see ADXL345_RA_OFSZ
 */
void getOffset(int8_t* x, int8_t* y, int8_t* z) {
    readBytes(devAddr_ADXL345, ADXL345_RA_OFSX, 3, buffer_ADXL345, 0);
    *x = buffer_ADXL345[0];
    *y = buffer_ADXL345[1];
    *z = buffer_ADXL345[2];
}
/** Set axis offsets.
 * @param x X axis offset value
 * @param y Y axis offset value
 * @param z Z axis offset value
 * @see getOffset()
 * @see ADXL345_RA_OFSX
 * @see ADXL345_RA_OFSY
 * @see ADXL345_RA_OFSZ
 */
void setOffset(int8_t x, int8_t y, int8_t z) {
    writeByte(devAddr_ADXL345, ADXL345_RA_OFSX, x);
    writeByte(devAddr_ADXL345, ADXL345_RA_OFSY, y);
    writeByte(devAddr_ADXL345, ADXL345_RA_OFSZ, z);
}
/** Get X axis offset.
 * @return X axis offset value
 * @see getOffset()
 * @see ADXL345_RA_OFSX
 */
int8_t getOffsetX() {
    readByte(devAddr_ADXL345, ADXL345_RA_OFSX, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set X axis offset.
 * @param x X axis offset value
 * @see getOffset()
 * @see ADXL345_RA_OFSX
 */
void setOffsetX(int8_t x) {
    writeByte(devAddr_ADXL345, ADXL345_RA_OFSX, x);
}
/** Get Y axis offset.
 * @return Y axis offset value
 * @see getOffset()
 * @see ADXL345_RA_OFSY
 */
int8_t getOffsetY() {
    readByte(devAddr_ADXL345, ADXL345_RA_OFSY, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set Y axis offset.
 * @param y Y axis offset value
 * @see getOffset()
 * @see ADXL345_RA_OFSY
 */
void setOffsetY(int8_t y) {
    writeByte(devAddr_ADXL345, ADXL345_RA_OFSY, y);
}
/** Get Z axis offset.
 * @return Z axis offset value
 * @see getOffset()
 * @see ADXL345_RA_OFSZ
 */
int8_t getOffsetZ() {
    readByte(devAddr_ADXL345, ADXL345_RA_OFSZ, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set Z axis offset.
 * @param z Z axis offset value
 * @see getOffset()
 * @see ADXL345_RA_OFSZ
 */
void setOffsetZ(int8_t z) {
    writeByte(devAddr_ADXL345, ADXL345_RA_OFSZ, z);
}

// DUR register

/** Get tap duration.
 * The DUR register is eight bits and contains an unsigned time value
 * representing the maximum time that an event must be above the THRESH_TAP
 * threshold to qualify as a tap event. The scale factor is 625 us/LSB. A value
 * of 0 disables the single tap/ double tap functions.
 * @return Tap duration (scaled at 625 us/LSB)
 * @see ADXL345_RA_DUR
 */
uint8_t getTapDuration() {
    readByte(devAddr_ADXL345, ADXL345_RA_DUR, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set tap duration.
 * @param duration Tap duration (scaled at 625 us/LSB)
 * @see getTapDuration()
 * @see ADXL345_RA_DUR
 */
void setTapDuration(uint8_t duration) {
    writeByte(devAddr_ADXL345, ADXL345_RA_DUR, duration);
}

// LATENT register

/** Get tap duration.
 * The latent register is eight bits and contains an unsigned time value
 * representing the wait time from the detection of a tap event to the start of
 * the time window (defined by the window register) during which a possible
 * second tap event can be detected. The scale factor is 1.25 ms/LSB. A value of
 * 0 disables the double tap function.
 * @return Tap latency (scaled at 1.25 ms/LSB)
 * @see ADXL345_RA_LATENT
 */
uint8_t getDoubleTapLatency() {
    readByte(devAddr_ADXL345, ADXL345_RA_LATENT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set tap duration.
 * @param latency Tap latency (scaled at 1.25 ms/LSB)
 * @see getDoubleTapLatency()
 * @see ADXL345_RA_LATENT
 */
void setDoubleTapLatency(uint8_t latency) {
    writeByte(devAddr_ADXL345, ADXL345_RA_LATENT, latency);
}

// WINDOW register

/** Get double tap window.
 * The window register is eight bits and contains an unsigned time value
 * representing the amount of time after the expiration of the latency time
 * (determined by the latent register) during which a second valid tap can
 * begin. The scale factor is 1.25 ms/LSB. A value of 0 disables the double tap
 * function.
 * @return Double tap window (scaled at 1.25 ms/LSB)
 * @see ADXL345_RA_WINDOW
 */
uint8_t getDoubleTapWindow() {
    readByte(devAddr_ADXL345, ADXL345_RA_WINDOW, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set double tap window.
 * @param window Double tap window (scaled at 1.25 ms/LSB)
 * @see getDoubleTapWindow()
 * @see ADXL345_RA_WINDOW
 */
void setDoubleTapWindow(uint8_t window) {
    writeByte(devAddr_ADXL345, ADXL345_RA_WINDOW, window);
}

// THRESH_ACT register

/** Get activity threshold.
 * The THRESH_ACT register is eight bits and holds the threshold value for
 * detecting activity. The data format is unsigned, so the magnitude of the
 * activity event is compared with the value in the THRESH_ACT register. The
 * scale factor is 62.5 mg/LSB. A value of 0 may result in undesirable behavior
 * if the activity interrupt is enabled.
 * @return Activity threshold (scaled at 62.5 mg/LSB)
 * @see ADXL345_RA_THRESH_ACT
 */
uint8_t getActivityThreshold() {
    readByte(devAddr_ADXL345, ADXL345_RA_THRESH_ACT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set activity threshold.
 * @param threshold Activity threshold (scaled at 62.5 mg/LSB)
 * @see getActivityThreshold()
 * @see ADXL345_RA_THRESH_ACT
 */
void setActivityThreshold(uint8_t threshold) {
    writeByte(devAddr_ADXL345, ADXL345_RA_THRESH_ACT, threshold);
}

// THRESH_INACT register

/** Get inactivity threshold.
 * The THRESH_INACT register is eight bits and holds the threshold value for 
 * detecting inactivity. The data format is unsigned, so the magnitude of the
 * inactivity event is compared with the value in the THRESH_INACT register. The
 * scale factor is 62.5 mg/LSB. A value of 0 may result in undesirable behavior
 * if the inactivity interrupt is enabled.
 * @return Inactivity threshold (scaled at 62.5 mg/LSB)
 * @see ADXL345_RA_THRESH_INACT
 */
uint8_t getInactivityThreshold() {
    readByte(devAddr_ADXL345, ADXL345_RA_THRESH_INACT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set inactivity threshold.
 * @param threshold Inctivity threshold (scaled at 62.5 mg/LSB)
 * @see getInctivityThreshold()
 * @see ADXL345_RA_THRESH_INACT
 */
void setInactivityThreshold(uint8_t threshold) {
    writeByte(devAddr_ADXL345, ADXL345_RA_THRESH_INACT, threshold);
}

// TIME_INACT register

/** Set inactivity time.
 * The TIME_INACT register is eight bits and contains an unsigned time value
 * representing the amount of time that acceleration must be less than the value
 * in the THRESH_INACT register for inactivity to be declared. The scale factor
 * is 1 sec/LSB. Unlike the other interrupt functions, which use unfiltered data
 * (see the Threshold sectionof the datasheet), the inactivity function uses
 * filtered output data. At least one output sample must be generated for the
 * inactivity interrupt to be triggered. This results in the function appearing
 * unresponsive if the TIME_INACT register is set to a value less than the time
 * constant of the output data rate. A value of 0 results in an interrupt when
 * the output data is less than the value in the THRESH_INACT register.
 * @return Inactivity time (scaled at 1 sec/LSB)
 * @see ADXL345_RA_TIME_INACT
 */
uint8_t getInactivityTime() {
    readByte(devAddr_ADXL345, ADXL345_RA_TIME_INACT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set inactivity time.
 * @param time Inactivity time (scaled at 1 sec/LSB)
 * @see getInctivityTime()
 * @see ADXL345_RA_TIME_INACT
 */
void setInactivityTime(uint8_t time) {
    writeByte(devAddr_ADXL345, ADXL345_RA_TIME_INACT, time);
}

// ACT_INACT_CTL register

/** Get activity AC/DC coupling.
 * A setting of 0 selects dc-coupled operation, and a setting of 1 enables
 * ac-coupled operation. In dc-coupled operation, the current acceleration
 * magnitude is compared directly with THRESH_ACT and THRESH_INACT to determine
 * whether activity or inactivity is detected.
 *
 * In ac-coupled operation for activity detection, the acceleration value at the
 * start of activity detection is taken as a reference value. New samples of
 * acceleration are then compared to this reference value, and if the magnitude
 * of the difference exceeds the THRESH_ACT value, the device triggers an
 * activity interrupt.
 *
 * Similarly, in ac-coupled operation for inactivity detection, a reference
 * value is used for comparison and is updated whenever the device exceeds the
 * inactivity threshold. After the reference value is selected, the device
 * compares the magnitude of the difference between the reference value and the
 * current acceleration with THRESH_INACT. If the difference is less than the
 * value in THRESH_INACT for the time in TIME_INACT, the device is considered
 * inactive and the inactivity interrupt is triggered.
 *
 * @return Activity coupling (0 = DC, 1 = AC)
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_ACT_AC_BIT
 */
int getActivityAC() {
    readBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_ACT_AC_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set activity AC/DC coupling.
 * @param enabled Activity AC/DC coupling (1 for AC, FALSE for DC)
 * @see getActivityAC()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_ACT_AC_BIT
 */
void setActivityAC(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_ACT_AC_BIT, enabled);
}
/** Get X axis activity monitoring inclusion.
 * For all "get[In]Activity*Enabled()" methods: a setting of 1 enables x-, y-,
 * or z-axis participation in detecting activity or inactivity. A setting of 0
 * excludes the selected axis from participation. If all axes are excluded, the
 * function is disabled. For activity detection, all participating axes are
 * logically OR�ed, causing the activity function to trigger when any of the
 * participating axes exceeds the threshold. For inactivity detection, all
 * participating axes are logically AND�ed, causing the inactivity function to
 * trigger only if all participating axes are below the threshold for the
 * specified time.
 * @return X axis activity monitoring enabled value
 * @see getActivityAC()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_ACT_X_BIT
 */
int getActivityXEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_ACT_X_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set X axis activity monitoring inclusion.
 * @param enabled X axis activity monitoring inclusion value
 * @see getActivityAC()
 * @see getActivityXEnabled()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_ACT_X_BIT
 */
void setActivityXEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_ACT_X_BIT, enabled);
}
/** Get Y axis activity monitoring.
 * @return Y axis activity monitoring enabled value
 * @see getActivityAC()
 * @see getActivityXEnabled()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_ACT_Y_BIT
 */
int getActivityYEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_ACT_Y_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set Y axis activity monitoring inclusion.
 * @param enabled Y axis activity monitoring inclusion value
 * @see getActivityAC()
 * @see getActivityXEnabled()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_ACT_Y_BIT
 */
void setActivityYEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_ACT_Y_BIT, enabled);
}
/** Get Z axis activity monitoring.
 * @return Z axis activity monitoring enabled value
 * @see getActivityAC()
 * @see getActivityXEnabled()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_ACT_Z_BIT
 */
int getActivityZEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_ACT_Z_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set Z axis activity monitoring inclusion.
 * @param enabled Z axis activity monitoring inclusion value
 * @see getActivityAC()
 * @see getActivityXEnabled()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_ACT_Z_BIT
 */
void setActivityZEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_ACT_Z_BIT, enabled);
}
/** Get inactivity AC/DC coupling.
 * @return Inctivity coupling (0 = DC, 1 = AC)
 * @see getActivityAC()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_INACT_AC_BIT
 */
int getInactivityAC() {
    readBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_INACT_AC_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set inctivity AC/DC coupling.
 * @param enabled Inactivity AC/DC coupling (1 for AC, FALSE for DC)
 * @see getActivityAC()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_INACT_AC_BIT
 */
void setInactivityAC(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_INACT_AC_BIT, enabled);
}
/** Get X axis inactivity monitoring.
 * @return Y axis inactivity monitoring enabled value
 * @see getActivityAC()
 * @see getActivityXEnabled()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_INACT_X_BIT
 */
int getInactivityXEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_INACT_X_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set X axis activity monitoring inclusion.
 * @param enabled X axis inactivity monitoring inclusion value
 * @see getActivityAC()
 * @see getActivityXEnabled()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_INACT_X_BIT
 */
void setInactivityXEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_INACT_X_BIT, enabled);
}
/** Get Y axis inactivity monitoring.
 * @return Y axis inactivity monitoring enabled value
 * @see getActivityAC()
 * @see getActivityXEnabled()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_INACT_Y_BIT
 */
int getInactivityYEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_INACT_Y_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set Y axis inactivity monitoring inclusion.
 * @param enabled Y axis inactivity monitoring inclusion value
 * @see getActivityAC()
 * @see getActivityXEnabled()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_INACT_Y_BIT
 */
void setInactivityYEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_INACT_Y_BIT, enabled);
}
/** Get Z axis inactivity monitoring.
 * @return Z axis inactivity monitoring enabled value
 * @see getActivityAC()
 * @see getActivityXEnabled()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_INACT_Z_BIT
 */
int getInactivityZEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_INACT_Z_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set Z axis inactivity monitoring inclusion.
 * @param enabled Z axis activity monitoring inclusion value
 * @see getActivityAC()
 * @see getActivityXEnabled()
 * @see ADXL345_RA_ACT_INACT_CTL
 * @see ADXL345_AIC_INACT_Z_BIT
 */
void setInactivityZEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_ACT_INACT_CTL, ADXL345_AIC_INACT_Z_BIT, enabled);
}

// THRESH_FF register

/** Get freefall threshold value.
 * The THRESH_FF register is eight bits and holds the threshold value, in
 * unsigned format, for free-fall detection. The acceleration on all axes is
 * compared with the value in THRESH_FF to determine if a free-fall event
 * occurred. The scale factor is 62.5 mg/LSB. Note that a value of 0 mg may
 * result in undesirable behavior if the free-fall interrupt is enabled. Values
 * between 300 mg and 600 mg (0x05 to 0x09) are recommended.
 * @return Freefall threshold value (scaled at 62.5 mg/LSB)
 * @see ADXL345_RA_THRESH_FF
 */
uint8_t getFreefallThreshold() {
    readByte(devAddr_ADXL345, ADXL345_RA_THRESH_FF, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set freefall threshold value.
 * @param threshold Freefall threshold value (scaled at 62.5 mg/LSB)
 * @see getFreefallThreshold()
 * @see ADXL345_RA_THRESH_FF
 */
void setFreefallThreshold(uint8_t threshold) {
    writeByte(devAddr_ADXL345, ADXL345_RA_THRESH_FF, threshold);
}

// TIME_FF register

/** Get freefall time value.
 * The TIME_FF register is eight bits and stores an unsigned time value
 * representing the minimum time that the value of all axes must be less than
 * THRESH_FF to generate a free-fall interrupt. The scale factor is 5 ms/LSB. A
 * value of 0 may result in undesirable behavior if the free-fall interrupt is
 * enabled. Values between 100 ms and 350 ms (0x14 to 0x46) are recommended.
 * @return Freefall time value (scaled at 5 ms/LSB)
 * @see getFreefallThreshold()
 * @see ADXL345_RA_TIME_FF
 */
uint8_t getFreefallTime() {
    readByte(devAddr_ADXL345, ADXL345_RA_TIME_FF, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set freefall time value.
 * @param threshold Freefall time value (scaled at 5 ms/LSB)
 * @see getFreefallTime()
 * @see ADXL345_RA_TIME_FF
 */
void setFreefallTime(uint8_t time) {
    writeByte(devAddr_ADXL345, ADXL345_RA_TIME_FF, time);
}

// TAP_AXES register

/** Get double-tap fast-movement suppression.
 * Setting the suppress bit suppresses double tap detection if acceleration
 * greater than the value in THRESH_TAP is present between taps. See the Tap
 * Detection section in the datasheet for more details.
 * @return Double-tap fast-movement suppression value
 * @see getTapThreshold()
 * @see ADXL345_RA_TAP_AXES
 * @see ADXL345_TAPAXIS_SUP_BIT
 */
int getTapAxisSuppress() {
    readBit(devAddr_ADXL345, ADXL345_RA_TAP_AXES, ADXL345_TAPAXIS_SUP_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set double-tap fast-movement suppression.
 * @param enabled Double-tap fast-movement suppression value
 * @see getTapAxisSuppress()
 * @see ADXL345_RA_TAP_AXES
 * @see ADXL345_TAPAXIS_SUP_BIT
 */
void setTapAxisSuppress(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_TAP_AXES, ADXL345_TAPAXIS_SUP_BIT, enabled);
}
/** Get double-tap fast-movement suppression.
 * A setting of 1 in the TAP_X enable bit enables x-axis participation in tap
 * detection. A setting of 0 excludes the selected axis from participation in
 * tap detection.
 * @return Double-tap fast-movement suppression value
 * @see getTapThreshold()
 * @see ADXL345_RA_TAP_AXES
 * @see ADXL345_TAPAXIS_X_BIT
 */
int getTapAxisXEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_TAP_AXES, ADXL345_TAPAXIS_X_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set tap detection X axis inclusion.
 * @param enabled X axis tap detection enabled value
 * @see getTapAxisXEnabled()
 * @see ADXL345_RA_TAP_AXES
 * @see ADXL345_TAPAXIS_X_BIT
 */
void setTapAxisXEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_TAP_AXES, ADXL345_TAPAXIS_X_BIT, enabled);
}
/** Get tap detection Y axis inclusion.
 * A setting of 1 in the TAP_Y enable bit enables y-axis participation in tap
 * detection. A setting of 0 excludes the selected axis from participation in
 * tap detection.
 * @return Double-tap fast-movement suppression value
 * @see getTapThreshold()
 * @see ADXL345_RA_TAP_AXES
 * @see ADXL345_TAPAXIS_Y_BIT
 */
int getTapAxisYEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_TAP_AXES, ADXL345_TAPAXIS_Y_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set tap detection Y axis inclusion.
 * @param enabled Y axis tap detection enabled value
 * @see getTapAxisYEnabled()
 * @see ADXL345_RA_TAP_AXES
 * @see ADXL345_TAPAXIS_Y_BIT
 */
void setTapAxisYEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_TAP_AXES, ADXL345_TAPAXIS_Y_BIT, enabled);
}
/** Get tap detection Z axis inclusion.
 * A setting of 1 in the TAP_Z enable bit enables z-axis participation in tap
 * detection. A setting of 0 excludes the selected axis from participation in
 * tap detection.
 * @return Double-tap fast-movement suppression value
 * @see getTapThreshold()
 * @see ADXL345_RA_TAP_AXES
 * @see ADXL345_TAPAXIS_Z_BIT
 */
int getTapAxisZEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_TAP_AXES, ADXL345_TAPAXIS_Z_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set tap detection Z axis inclusion.
 * @param enabled Z axis tap detection enabled value
 * @see getTapAxisZEnabled()
 * @see ADXL345_RA_TAP_AXES
 * @see ADXL345_TAPAXIS_Z_BIT
 */
void setTapAxisZEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_TAP_AXES, ADXL345_TAPAXIS_Z_BIT, enabled);
}

// ACT_TAP_STATUS register

/** Get X axis activity source flag.
 * These bits indicate the first axis involved in a tap or activity event. A
 * setting of 1 corresponds to involvement in the event, and a setting of 0
 * corresponds to no involvement. When new data is available, these bits are not
 * cleared but are overwritten by the new data. The ACT_TAP_STATUS register
 * should be read before clearing the interrupt. Disabling an axis from
 * participation clears the corresponding source bit when the next activity or
 * single tap/double tap event occurs.
 * @return X axis activity source flag
 * @see ADXL345_RA_ACT_TAP_STATUS
 * @see ADXL345_TAPSTAT_ACTX_BIT
 */
int getActivitySourceX() {
    readBit(devAddr_ADXL345, ADXL345_RA_ACT_TAP_STATUS, ADXL345_TAPSTAT_ACTX_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Get Y axis activity source flag.
 * @return Y axis activity source flag
 * @see getActivitySourceX()
 * @see ADXL345_RA_ACT_TAP_STATUS
 * @see ADXL345_TAPSTAT_ACTY_BIT
 */
int getActivitySourceY() {
    readBit(devAddr_ADXL345, ADXL345_RA_ACT_TAP_STATUS, ADXL345_TAPSTAT_ACTY_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Get Z axis activity source flag.
 * @return Z axis activity source flag
 * @see getActivitySourceX()
 * @see ADXL345_RA_ACT_TAP_STATUS
 * @see ADXL345_TAPSTAT_ACTZ_BIT
 */
int getActivitySourceZ() {
    readBit(devAddr_ADXL345, ADXL345_RA_ACT_TAP_STATUS, ADXL345_TAPSTAT_ACTZ_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Get sleep mode flag.
 * A setting of 1 in the asleep bit indicates that the part is asleep, and a
 * setting of 0 indicates that the part is not asleep. This bit toggles only if
 * the device is configured for auto sleep. See the AUTO_SLEEP Bit section of
 * the datasheet for more information on autosleep mode.
 * @return Sleep mode enabled flag
 * @see ADXL345_RA_ACT_TAP_STATUS
 * @see ADXL345_TAPSTAT_ASLEEP_BIT
 */
int getAsleep() {
    readBit(devAddr_ADXL345, ADXL345_RA_ACT_TAP_STATUS, ADXL345_TAPSTAT_ASLEEP_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Get X axis tap source flag.
 * @return X axis tap source flag
 * @see getActivitySourceX()
 * @see ADXL345_RA_ACT_TAP_STATUS
 * @see ADXL345_TAPSTAT_TAPX_BIT
 */
int getTapSourceX() {
    readBit(devAddr_ADXL345, ADXL345_RA_ACT_TAP_STATUS, ADXL345_TAPSTAT_TAPX_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Get Y axis tap source flag.
 * @return Y axis tap source flag
 * @see getActivitySourceX()
 * @see ADXL345_RA_ACT_TAP_STATUS
 * @see ADXL345_TAPSTAT_TAPY_BIT
 */
int getTapSourceY() {
    readBit(devAddr_ADXL345, ADXL345_RA_ACT_TAP_STATUS, ADXL345_TAPSTAT_TAPY_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Get Z axis tap source flag.
 * @return Z axis tap source flag
 * @see getActivitySourceX()
 * @see ADXL345_RA_ACT_TAP_STATUS
 * @see ADXL345_TAPSTAT_TAPZ_BIT
 */
int getTapSourceZ() {
    readBit(devAddr_ADXL345, ADXL345_RA_ACT_TAP_STATUS, ADXL345_TAPSTAT_TAPZ_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}

// BW_RATE register

/** Get low power enabled status.
 * A setting of 0 in the LOW_POWER bit selects normal operation, and a setting
 * of 1 selects reduced power operation, which has somewhat higher noise (see
 * the Power Modes section of the datasheet for details).
 * @return Low power enabled status
 * @see ADXL345_RA_BW_RATE
 * @see ADXL345_BW_LOWPOWER_BIT
 */
int getLowPowerEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_BW_RATE, ADXL345_BW_LOWPOWER_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set low power enabled status.
 * @see getLowPowerEnabled()
 * @param enabled Low power enable setting
 * @see ADXL345_RA_BW_RATE
 * @see ADXL345_BW_LOWPOWER_BIT
 */
void setLowPowerEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_BW_RATE, ADXL345_BW_LOWPOWER_BIT, enabled);
}
/** Get measurement data rate.
 * These bits select the device bandwidth and output data rate (see Table 7 and
 * Table 8 in the datasheet for details). The default value is 0x0A, which
 * translates to a 100 Hz output data rate. An output data rate should be
 * selected that is appropriate for the communication protocol and frequency
 * selected. Selecting too high of an output data rate with a low communication
 * speed results in samples being discarded.
 * @return Data rate (0x0 - 0xF)
 * @see ADXL345_RA_BW_RATE
 * @see ADXL345_BW_RATE_BIT
 * @see ADXL345_BW_RATE_LENGTH
 */
uint8_t getRate() {
    readBits(devAddr_ADXL345, ADXL345_RA_BW_RATE, ADXL345_BW_RATE_BIT, ADXL345_BW_RATE_LENGTH, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set measurement data rate.
 * 0x7 =  12.5Hz
 * 0x8 =  25Hz, increasing or decreasing by factors of 2, so:
 * 0x9 =  50Hz
 * 0xA = 100Hz
 * @param rate New data rate (0x0 - 0xF)
 * @see ADXL345_RATE_100
 * @see ADXL345_RA_BW_RATE
 * @see ADXL345_BW_RATE_BIT
 * @see ADXL345_BW_RATE_LENGTH
 */
void setRate(uint8_t rate) {
    writeBits(devAddr_ADXL345, ADXL345_RA_BW_RATE, ADXL345_BW_RATE_BIT, ADXL345_BW_RATE_LENGTH, rate);
}

// POWER_CTL register

/** Get activity/inactivity serial linkage status.
 * A setting of 1 in the link bit with both the activity and inactivity
 * functions enabled delays the start of the activity function until
 * inactivity is detected. After activity is detected, inactivity detection
 * begins, preventing the detection of activity. This bit serially links the
 * activity and inactivity functions. When this bit is set to 0, the inactivity
 * and activity functions are concurrent. Additional information can be found
 * in the Link Mode section of the datasheet.
 *
 * When clearing the link bit, it is recommended that the part be placed into
 * standby mode and then set back to measurement mode with a subsequent write.
 * This is done to ensure that the device is properly biased if sleep mode is
 * manually disabled; otherwise, the first few samples of data after the link
 * bit is cleared may have additional noise, especially if the device was asleep
 * when the bit was cleared.
 *
 * @return Link status
 * @see ADXL345_RA_POWER_CTL
 * @see ADXL345_PCTL_LINK_BIT
 */
int getLinkEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_POWER_CTL, ADXL345_PCTL_LINK_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set activity/inactivity serial linkage status.
 * @param enabled New link status
 * @see ADXL345_RA_POWER_CTL
 * @see ADXL345_PCTL_LINK_BIT
 */
void setLinkEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_POWER_CTL, ADXL345_PCTL_LINK_BIT, enabled);
}
/** Get auto-sleep enabled status.
 * If the link bit is set, a setting of 1 in the AUTO_SLEEP bit enables the
 * auto-sleep functionality. In this mode, the ADXL345 auto-matically switches
 * to sleep mode if the inactivity function is enabled and inactivity is
 * detected (that is, when acceleration is below the THRESH_INACT value for at
 * least the time indicated by TIME_INACT). If activity is also enabled, the
 * ADXL345 automatically wakes up from sleep after detecting activity and
 * returns to operation at the output data rate set in the BW_RATE register. A
 * setting of 0 in the AUTO_SLEEP bit disables automatic switching to sleep
 * mode. See the description of the Sleep Bit in this section of the datasheet
 * for more information on sleep mode.
 *
 * If the link bit is not set, the AUTO_SLEEP feature is disabled and setting
 * the AUTO_SLEEP bit does not have an impact on device operation. Refer to the
 * Link Bit section or the Link Mode section for more information on utilization
 * of the link feature.
 *
 * When clearing the AUTO_SLEEP bit, it is recommended that the part be placed
 * into standby mode and then set back to measure-ment mode with a subsequent
 * write. This is done to ensure that the device is properly biased if sleep
 * mode is manually disabled; otherwise, the first few samples of data after the
 * AUTO_SLEEP bit is cleared may have additional noise, especially if the device
 * was asleep when the bit was cleared.
 *
 * @return Auto-sleep enabled status
 * @see getActivityThreshold()
 * @see getInactivityThreshold()
 * @see getInactivityTime()
 * @see ADXL345_RA_POWER_CTL
 * @see ADXL345_PCTL_AUTOSLEEP_BIT
 */
int getAutoSleepEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_POWER_CTL, ADXL345_PCTL_AUTOSLEEP_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set auto-sleep enabled status.
 * @param enabled New auto-sleep status
 * @see getAutoSleepEnabled()
 * @see ADXL345_RA_POWER_CTL
 * @see ADXL345_PCTL_AUTOSLEEP_BIT
 */
void setAutoSleepEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_POWER_CTL, ADXL345_PCTL_AUTOSLEEP_BIT, enabled);
}
/** Get measurement enabled status.
 * A setting of 0 in the measure bit places the part into standby mode, and a
 * setting of 1 places the part into measurement mode. The ADXL345 powers up in
 * standby mode with minimum power consumption.
 * @return Measurement enabled status
 * @see ADXL345_RA_POWER_CTL
 * @see ADXL345_PCTL_MEASURE_BIT
 */
int getMeasureEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_POWER_CTL, ADXL345_PCTL_MEASURE_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set measurement enabled status.
 * @param enabled Measurement enabled status
 * @see getMeasureEnabled()
 * @see ADXL345_RA_POWER_CTL
 * @see ADXL345_PCTL_MEASURE_BIT
 */
void setMeasureEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_POWER_CTL, ADXL345_PCTL_MEASURE_BIT, enabled);
}
/** Get sleep mode enabled status.
 * A setting of 0 in the sleep bit puts the part into the normal mode of
 * operation, and a setting of 1 places the part into sleep mode. Sleep mode
 * suppresses DATA_READY, stops transmission of data to FIFO, and switches the
 * sampling rate to one specified by the wakeup bits. In sleep mode, only the
 * activity function can be used. When the DATA_READY interrupt is suppressed,
 * the output data registers (Register 0x32 to Register 0x37) are still updated
 * at the sampling rate set by the wakeup bits (D1:D0).
 *
 * When clearing the sleep bit, it is recommended that the part be placed into
 * standby mode and then set back to measurement mode with a subsequent write.
 * This is done to ensure that the device is properly biased if sleep mode is
 * manually disabled; otherwise, the first few samples of data after the sleep
 * bit is cleared may have additional noise, especially if the device was asleep
 * when the bit was cleared.
 *
 * @return Sleep enabled status
 * @see ADXL345_RA_POWER_CTL
 * @see ADXL345_PCTL_SLEEP_BIT
 */
int getSleepEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_POWER_CTL, ADXL345_PCTL_SLEEP_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set sleep mode enabled status.
 * @param Sleep mode enabled status
 * @see getSleepEnabled()
 * @see ADXL345_RA_POWER_CTL
 * @see ADXL345_PCTL_SLEEP_BIT
 */
void setSleepEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_POWER_CTL, ADXL345_PCTL_SLEEP_BIT, enabled);
}
/** Get wakeup frequency.
 * These bits control the frequency of readings in sleep mode as described in
 * Table 20 in the datasheet. (That is, 0 = 8Hz, 1 = 4Hz, 2 = 2Hz, 3 = 1Hz)
 * @return Wakeup frequency (0x0 - 0x3, indicating 8/4/2/1Hz respectively)
 * @see ADXL345_RA_POWER_CTL
 * @see ADXL345_PCTL_SLEEP_BIT
 */
uint8_t getWakeupFrequency() {
    readBits(devAddr_ADXL345, ADXL345_RA_POWER_CTL, ADXL345_PCTL_WAKEUP_BIT, ADXL345_PCTL_WAKEUP_LENGTH, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set wakeup frequency.
 * @param frequency Wakeup frequency (0x0 - 0x3, indicating 8/4/2/1Hz respectively)
 * @see getWakeupFrequency()
 * @see ADXL345_RA_POWER_CTL
 * @see ADXL345_PCTL_SLEEP_BIT
 */
void setWakeupFrequency(uint8_t frequency) {
    writeBits(devAddr_ADXL345, ADXL345_RA_POWER_CTL, ADXL345_PCTL_WAKEUP_BIT, ADXL345_PCTL_WAKEUP_LENGTH, frequency);
}

// INT_ENABLE register

/** Get DATA_READY interrupt enabled status.
 * Setting bits in this register to a value of 1 enables their respective
 * functions to generate interrupts, whereas a value of 0 prevents the functions
 * from generating interrupts. The DATA_READY, watermark, and overrun bits
 * enable only the interrupt output; the functions are always enabled. It is
 * recommended that interrupts be configured before enabling their outputs.
 * @return DATA_READY interrupt enabled status.
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_DATA_READY_BIT
 */
int getIntDataReadyEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_DATA_READY_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set DATA_READY interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_DATA_READY_BIT
 */
void setIntDataReadyEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_DATA_READY_BIT, enabled);
}
/** Set SINGLE_TAP interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_SINGLE_TAP_BIT
 */
int getIntSingleTapEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_SINGLE_TAP_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set SINGLE_TAP interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_SINGLE_TAP_BIT
 */
void setIntSingleTapEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_SINGLE_TAP_BIT, enabled);
}
/** Get DOUBLE_TAP interrupt enabled status.
 * @return Interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_DOUBLE_TAP_BIT
 */
int getIntDoubleTapEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_DOUBLE_TAP_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set DOUBLE_TAP interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_DOUBLE_TAP_BIT
 */
void setIntDoubleTapEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_DOUBLE_TAP_BIT, enabled);
}
/** Set ACTIVITY interrupt enabled status.
 * @return Interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_ACTIVITY_BIT
 */
int getIntActivityEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_ACTIVITY_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set ACTIVITY interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_ACTIVITY_BIT
 */
void setIntActivityEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_ACTIVITY_BIT, enabled);
}
/** Get INACTIVITY interrupt enabled status.
 * @return Interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_INACTIVITY_BIT
 */
int getIntInactivityEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_INACTIVITY_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set INACTIVITY interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_INACTIVITY_BIT
 */
void setIntInactivityEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_INACTIVITY_BIT, enabled);
}
/** Get FREE_FALL interrupt enabled status.
 * @return Interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_FREE_FALL_BIT
 */
int getIntFreefallEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_FREE_FALL_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set FREE_FALL interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_FREE_FALL_BIT
 */
void setIntFreefallEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_FREE_FALL_BIT, enabled);
}
/** Get WATERMARK interrupt enabled status.
 * @return Interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_WATERMARK_BIT
 */
int getIntWatermarkEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_WATERMARK_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set WATERMARK interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_WATERMARK_BIT
 */
void setIntWatermarkEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_WATERMARK_BIT, enabled);
}
/** Get OVERRUN interrupt enabled status.
 * @return Interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_OVERRUN_BIT
 */
int getIntOverrunEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_OVERRUN_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set OVERRUN interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see getIntDataReadyEnabled()
 * @see ADXL345_RA_INT_ENABLE
 * @see ADXL345_INT_OVERRUN_BIT
 */
void setIntOverrunEnabled(int enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_ENABLE, ADXL345_INT_OVERRUN_BIT, enabled);
}

// INT_MAP register

/** Get DATA_READY interrupt pin.
 * Any bits set to 0 in this register send their respective interrupts to the
 * INT1 pin, whereas bits set to 1 send their respective interrupts to the INT2
 * pin. All selected interrupts for a given pin are OR'ed.
 * @return Interrupt pin setting
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_DATA_READY_BIT
 */
uint8_t getIntDataReadyPin() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_DATA_READY_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set DATA_READY interrupt pin.
 * @param pin Interrupt pin setting
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_DATA_READY_BIT
 */
void setIntDataReadyPin(uint8_t pin) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_DATA_READY_BIT, pin);
}
/** Get SINGLE_TAP interrupt pin.
 * @return Interrupt pin setting
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_SINGLE_TAP_BIT
 */
uint8_t getIntSingleTapPin() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_SINGLE_TAP_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set SINGLE_TAP interrupt pin.
 * @param pin Interrupt pin setting
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_SINGLE_TAP_BIT
 */
void setIntSingleTapPin(uint8_t pin) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_SINGLE_TAP_BIT, pin);
}
/** Get DOUBLE_TAP interrupt pin.
 * @return Interrupt pin setting
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_DOUBLE_TAP_BIT
 */
uint8_t getIntDoubleTapPin() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_DOUBLE_TAP_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set DOUBLE_TAP interrupt pin.
 * @param pin Interrupt pin setting
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_DOUBLE_TAP_BIT
 */
void setIntDoubleTapPin(uint8_t pin) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_DOUBLE_TAP_BIT, pin);
}
/** Get ACTIVITY interrupt pin.
 * @return Interrupt pin setting
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_ACTIVITY_BIT
 */
uint8_t getIntActivityPin() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_ACTIVITY_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set ACTIVITY interrupt pin.
 * @param pin Interrupt pin setting
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_ACTIVITY_BIT
 */
void setIntActivityPin(uint8_t pin) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_ACTIVITY_BIT, pin);
}
/** Get INACTIVITY interrupt pin.
 * @return Interrupt pin setting
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_INACTIVITY_BIT
 */
uint8_t getIntInactivityPin() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_INACTIVITY_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set INACTIVITY interrupt pin.
 * @param pin Interrupt pin setting
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_INACTIVITY_BIT
 */
void setIntInactivityPin(uint8_t pin) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_INACTIVITY_BIT, pin);
}
/** Get FREE_FALL interrupt pin.
 * @return Interrupt pin setting
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_FREE_FALL_BIT
 */
uint8_t getIntFreefallPin() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_FREE_FALL_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set FREE_FALL interrupt pin.
 * @param pin Interrupt pin setting
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_FREE_FALL_BIT
 */
void setIntFreefallPin(uint8_t pin) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_FREE_FALL_BIT, pin);
}
/** Get WATERMARK interrupt pin.
 * @return Interrupt pin setting
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_WATERMARK_BIT
 */
uint8_t getIntWatermarkPin() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_WATERMARK_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set WATERMARK interrupt pin.
 * @param pin Interrupt pin setting
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_WATERMARK_BIT
 */
void setIntWatermarkPin(uint8_t pin) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_WATERMARK_BIT, pin);
}
/** Get OVERRUN interrupt pin.
 * @return Interrupt pin setting
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_OVERRUN_BIT
 */
uint8_t getIntOverrunPin() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_OVERRUN_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set OVERRUN interrupt pin.
 * @param pin Interrupt pin setting
 * @see getIntDataReadyPin()
 * @see ADXL345_RA_INT_MAP
 * @see ADXL345_INT_OVERRUN_BIT
 */
void setIntOverrunPin(uint8_t pin) {
    writeBit(devAddr_ADXL345, ADXL345_RA_INT_MAP, ADXL345_INT_OVERRUN_BIT, pin);
}

// INT_SOURCE register

/** Get DATA_READY interrupt source flag.
 * Bits set to 1 in this register indicate that their respective functions have
 * triggered an event, whereas a value of 0 indicates that the corresponding
 * event has not occurred. The DATA_READY, watermark, and overrun bits are
 * always set if the corresponding events occur, regardless of the INT_ENABLE
 * register settings, and are cleared by reading data from the DATAX, DATAY, and
 * DATAZ registers. The DATA_READY and watermark bits may require multiple
 * reads, as indicated in the FIFO mode descriptions in the FIFO section. Other
 * bits, and the corresponding interrupts, are cleared by reading the INT_SOURCE
 * register.
 * @return Interrupt source flag
 * @see ADXL345_RA_INT_SOURCE
 * @see ADXL345_INT_DATA_READY_BIT
 */
uint8_t getIntDataReadySource() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_SOURCE, ADXL345_INT_DATA_READY_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Get SINGLE_TAP interrupt source flag.
 * @return Interrupt source flag
 * @see ADXL345_RA_INT_SOURCE
 * @see ADXL345_INT_SINGLE_TAP_BIT
 */
uint8_t getIntSingleTapSource() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_SOURCE, ADXL345_INT_SINGLE_TAP_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Get DOUBLE_TAP interrupt source flag.
 * @return Interrupt source flag
 * @see ADXL345_RA_INT_SOURCE
 * @see ADXL345_INT_DOUBLE_TAP_BIT
 */
uint8_t getIntDoubleTapSource() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_SOURCE, ADXL345_INT_DOUBLE_TAP_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Get ACTIVITY interrupt source flag.
 * @return Interrupt source flag
 * @see ADXL345_RA_INT_SOURCE
 * @see ADXL345_INT_ACTIVITY_BIT
 */
uint8_t getIntActivitySource() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_SOURCE, ADXL345_INT_ACTIVITY_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Get INACTIVITY interrupt source flag.
 * @return Interrupt source flag
 * @see ADXL345_RA_INT_SOURCE
 * @see ADXL345_INT_INACTIVITY_BIT
 */
uint8_t getIntInactivitySource() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_SOURCE, ADXL345_INT_INACTIVITY_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Get FREE_FALL interrupt source flag.
 * @return Interrupt source flag
 * @see ADXL345_RA_INT_SOURCE
 * @see ADXL345_INT_FREE_FALL_BIT
 */
uint8_t getIntFreefallSource() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_SOURCE, ADXL345_INT_FREE_FALL_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Get WATERMARK interrupt source flag.
 * @return Interrupt source flag
 * @see ADXL345_RA_INT_SOURCE
 * @see ADXL345_INT_WATERMARK_BIT
 */
uint8_t getIntWatermarkSource() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_SOURCE, ADXL345_INT_WATERMARK_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Get OVERRUN interrupt source flag.
 * @return Interrupt source flag
 * @see ADXL345_RA_INT_SOURCE
 * @see ADXL345_INT_OVERRUN_BIT
 */
uint8_t getIntOverrunSource() {
    readBit(devAddr_ADXL345, ADXL345_RA_INT_SOURCE, ADXL345_INT_OVERRUN_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}

// DATA_FORMAT register

/** Get self-test force enabled.
 * A setting of 1 in the SELF_TEST bit applies a self-test force to the sensor,
 * causing a shift in the output data. A value of 0 disables the self-test
 * force.
 * @return Self-test force enabled setting
 * @see ADXL345_RA_DATA_FORMAT
 * @see ADXL345_FORMAT_SELFTEST_BIT
 */
uint8_t getSelfTestEnabled() {
    readBit(devAddr_ADXL345, ADXL345_RA_DATA_FORMAT, ADXL345_FORMAT_SELFTEST_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set self-test force enabled.
 * @param enabled New self-test force enabled setting
 * @see getSelfTestEnabled()
 * @see ADXL345_RA_DATA_FORMAT
 * @see ADXL345_FORMAT_SELFTEST_BIT
 */
void setSelfTestEnabled(uint8_t enabled) {
    writeBit(devAddr_ADXL345, ADXL345_RA_DATA_FORMAT, ADXL345_FORMAT_SELFTEST_BIT, enabled);
}
/** Get SPI mode setting.
 * A value of 1 in the SPI bit sets the device to 3-wire SPI mode, and a value
 * of 0 sets the device to 4-wire SPI mode.
 * @return SPI mode setting
 * @see ADXL345_RA_DATA_FORMAT
 * @see ADXL345_FORMAT_SELFTEST_BIT
 */
uint8_t getSPIMode_ADXK345() {
    readBit(devAddr_ADXL345, ADXL345_RA_DATA_FORMAT, ADXL345_FORMAT_SPIMODE_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set SPI mode setting.
 * @param mode New SPI mode setting
 * @see getSPIMode()
 * @see ADXL345_RA_DATA_FORMAT
 * @see ADXL345_FORMAT_SELFTEST_BIT
 */
void setSPIMode_ADXL345(uint8_t mode) {
    writeBit(devAddr_ADXL345, ADXL345_RA_DATA_FORMAT, ADXL345_FORMAT_SPIMODE_BIT, mode);
}
/** Get interrupt mode setting.
 * A value of 0 in the INT_INVERT bit sets the interrupts to active high, and a
 * value of 1 sets the interrupts to active low.
 * @return Interrupt mode setting
 * @see ADXL345_RA_DATA_FORMAT
 * @see ADXL345_FORMAT_INTMODE_BIT
 */
uint8_t getInterruptMode() {
    readBit(devAddr_ADXL345, ADXL345_RA_DATA_FORMAT, ADXL345_FORMAT_INTMODE_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set interrupt mode setting.
 * @param mode New interrupt mode setting
 * @see getInterruptMode()
 * @see ADXL345_RA_DATA_FORMAT
 * @see ADXL345_FORMAT_INTMODE_BIT
 */
void setInterruptMode(uint8_t mode) {
    writeBit(devAddr_ADXL345, ADXL345_RA_DATA_FORMAT, ADXL345_FORMAT_INTMODE_BIT, mode);
}
/** Get full resolution mode setting.
 * When this bit is set to a value of 1, the device is in full resolution mode,
 * where the output resolution increases with the g range set by the range bits
 * to maintain a 4 mg/LSB scale factor. When the FULL_RES bit is set to 0, the
 * device is in 10-bit mode, and the range bits determine the maximum g range
 * and scale factor.
 * @return Full resolution enabled setting
 * @see ADXL345_RA_DATA_FORMAT
 * @see ADXL345_FORMAT_FULL_RES_BIT
 */
uint8_t getFullResolution() {
    readBit(devAddr_ADXL345, ADXL345_RA_DATA_FORMAT, ADXL345_FORMAT_FULL_RES_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set full resolution mode setting.
 * @param resolution New full resolution enabled setting
 * @see getFullResolution()
 * @see ADXL345_RA_DATA_FORMAT
 * @see ADXL345_FORMAT_FULL_RES_BIT
 */
void setFullResolution(uint8_t resolution) {
    writeBit(devAddr_ADXL345, ADXL345_RA_DATA_FORMAT, ADXL345_FORMAT_FULL_RES_BIT, resolution);
}
/** Get data justification mode setting.
 * A setting of 1 in the justify bit selects left-justified (MSB) mode, and a
 * setting of 0 selects right-justified mode with sign extension.
 * @return Data justification mode
 * @see ADXL345_RA_DATA_FORMAT
 * @see ADXL345_FORMAT_JUSTIFY_BIT
 */
uint8_t getDataJustification() {
    readBit(devAddr_ADXL345, ADXL345_RA_DATA_FORMAT, ADXL345_FORMAT_JUSTIFY_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set data justification mode setting.
 * @param justification New data justification mode
 * @see getDataJustification()
 * @see ADXL345_RA_DATA_FORMAT
 * @see ADXL345_FORMAT_JUSTIFY_BIT
 */
void setDataJustification(uint8_t justification) {
    writeBit(devAddr_ADXL345, ADXL345_RA_DATA_FORMAT, ADXL345_FORMAT_JUSTIFY_BIT, justification);
}
/** Get data range setting.
 * These bits set the g range as described in Table 21. (That is, 0x0 - 0x3 to
 * indicate 2g/4g/8g/16g respectively)
 * @return Range value (0x0 - 0x3 for 2g/4g/8g/16g)
 * @see ADXL345_RA_DATA_FORMAT
 * @see ADXL345_FORMAT_RANGE_BIT
 * @see ADXL345_FORMAT_RANGE_LENGTH
 */
uint8_t getRange() {
    readBits(devAddr_ADXL345, ADXL345_RA_DATA_FORMAT, ADXL345_FORMAT_RANGE_BIT, ADXL345_FORMAT_RANGE_LENGTH, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set data range setting.
 * @param range Range value (0x0 - 0x3 for 2g/4g/8g/16g)
 * @see getRange()
 * @see ADXL345_RA_DATA_FORMAT
 * @see ADXL345_FORMAT_RANGE_BIT
 * @see ADXL345_FORMAT_RANGE_LENGTH
 */
void setRange(uint8_t range) {
    writeBits(devAddr_ADXL345, ADXL345_RA_DATA_FORMAT, ADXL345_FORMAT_RANGE_BIT, ADXL345_FORMAT_RANGE_LENGTH, range);
}

// DATA* registers

/** Get 3-axis accleration measurements.
 * These six bytes (Register 0x32 to Register 0x37) are eight bits each and hold
 * the output data for each axis. Register 0x32 and Register 0x33 hold the
 * output data for the x-axis, Register 0x34 and Register 0x35 hold the output
 * data for the y-axis, and Register 0x36 and Register 0x37 hold the output data
 * for the z-axis. The output data is twos complement, with DATAx0 as the least
 * significant byte and DATAx1 as the most significant byte, where x represent
 * X, Y, or Z. The DATA_FORMAT register (Address 0x31) controls the format of
 * the data. It is recommended that a multiple-byte read of all registers be
 * performed to prevent a change in data between reads of sequential registers.
 * 
 * The DATA_FORMAT register controls the presentation of data to Register 0x32
 * through Register 0x37. All data, except that for the +/-16 g range, must be
 * clipped to avoid rollover.
 *
 * @param x 16-bit signed integer container for X-axis acceleration
 * @param y 16-bit signed integer container for Y-axis acceleration
 * @param z 16-bit signed integer container for Z-axis acceleration
 * @see ADXL345_RA_DATAX0
 */
void getAcceleration(int16_t* x, int16_t* y, int16_t* z) {
    readBytes(devAddr_ADXL345, ADXL345_RA_DATAX0, 6, buffer_ADXL345, 0);
    *x = (((int16_t)buffer_ADXL345[1]) << 8) | buffer_ADXL345[0];
    *y = (((int16_t)buffer_ADXL345[3]) << 8) | buffer_ADXL345[2];
    *z = (((int16_t)buffer_ADXL345[5]) << 8) | buffer_ADXL345[4];
}
/** Get X-axis accleration measurement.
 * @return 16-bit signed X-axis acceleration value
 * @see ADXL345_RA_DATAX0
 */
int16_t getAccelerationX() {
    readBytes(devAddr_ADXL345, ADXL345_RA_DATAX0, 2, buffer_ADXL345, 0);
    return (((int16_t)buffer_ADXL345[1]) << 8) | buffer_ADXL345[0];
}
/** Get Y-axis accleration measurement.
 * @return 16-bit signed Y-axis acceleration value
 * @see ADXL345_RA_DATAY0
 */
int16_t getAccelerationY() {
    readBytes(devAddr_ADXL345, ADXL345_RA_DATAY0, 2, buffer_ADXL345, 0);
    return (((int16_t)buffer_ADXL345[1]) << 8) | buffer_ADXL345[0];
}
/** Get Z-axis accleration measurement.
 * @return 16-bit signed Z-axis acceleration value
 * @see ADXL345_RA_DATAZ0
 */
int16_t getAccelerationZ() {
    readBytes(devAddr_ADXL345, ADXL345_RA_DATAZ0, 2, buffer_ADXL345, 0);
    return (((int16_t)buffer_ADXL345[1]) << 8) | buffer_ADXL345[0];
}

// FIFO_CTL register

/** Get FIFO mode.
 * These bits set the FIFO mode, as described in Table 22. That is:
 *
 * 0x0 = Bypass (FIFO is bypassed.)
 *
 * 0x1 = FIFO (FIFO collects up to 32 values and then stops collecting data,
 *       collecting new data only when FIFO is not full.)
 *
 * 0x2 = Stream (FIFO holds the last 32 data values. When FIFO is full, the
 *       oldest data is overwritten with newer data.)
 *
 * 0x3 = Trigger (When triggered by the trigger bit, FIFO holds the last data
 *       samples before the trigger event and then continues to collect data 
 *       until full. New data is collected only when FIFO is not full.)
 *
 * @return Curent FIFO mode
 * @see ADXL345_RA_FIFO_CTL
 * @see ADXL345_FIFO_MODE_BIT
 * @see ADXL345_FIFO_MODE_LENGTH
 */
uint8_t getFIFOMode_ADXL345() {
    readBits(devAddr_ADXL345, ADXL345_RA_FIFO_CTL, ADXL345_FIFO_MODE_BIT, ADXL345_FIFO_MODE_LENGTH, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set FIFO mode.
 * @param mode New FIFO mode
 * @see getFIFOMode()
 * @see ADXL345_RA_FIFO_CTL
 * @see ADXL345_FIFO_MODE_BIT
 * @see ADXL345_FIFO_MODE_LENGTH
 */
void setFIFOMode_ADXL345(uint8_t mode) {
    writeBits(devAddr_ADXL345, ADXL345_RA_FIFO_CTL, ADXL345_FIFO_MODE_BIT, ADXL345_FIFO_MODE_LENGTH, mode);
}
/** Get FIFO trigger interrupt setting.
 * A value of 0 in the trigger bit links the trigger event of trigger mode to
 * INT1, and a value of 1 links the trigger event to INT2.
 * @return Current FIFO trigger interrupt setting
 * @see ADXL345_RA_FIFO_CTL
 * @see ADXL345_FIFO_TRIGGER_BIT
 */
uint8_t getFIFOTriggerInterruptPin() {
    readBit(devAddr_ADXL345, ADXL345_RA_FIFO_CTL, ADXL345_FIFO_TRIGGER_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set FIFO trigger interrupt pin setting.
 * @param interrupt New FIFO trigger interrupt pin setting
 * @see ADXL345_RA_FIFO_CTL
 * @see ADXL345_FIFO_TRIGGER_BIT
 */
void setFIFOTriggerInterruptPin(uint8_t interrupt) {
    writeBit(devAddr_ADXL345, ADXL345_RA_FIFO_CTL, ADXL345_FIFO_TRIGGER_BIT, interrupt);
}
/** Get FIFO samples setting.
 * The function of these bits depends on the FIFO mode selected (see Table 23).
 * Entering a value of 0 in the samples bits immediately sets the watermark
 * status bit in the INT_SOURCE register, regardless of which FIFO mode is
 * selected. Undesirable operation may occur if a value of 0 is used for the
 * samples bits when trigger mode is used.
 *
 * MODE    | EFFECT
 * --------+-------------------------------------------------------------------
 * Bypass  | None.
 * FIFO    | FIFO entries needed to trigger a watermark interrupt.
 * Stream  | FIFO entries needed to trigger a watermark interrupt.
 * Trigger | Samples are retained in the FIFO buffer_ADXL345 before a trigger event.
 *
 * @return Current FIFO samples setting
 * @see ADXL345_RA_FIFO_CTL
 * @see ADXL345_FIFO_SAMPLES_BIT
 * @see ADXL345_FIFO_SAMPLES_LENGTH
 */
uint8_t getFIFOSamples() {
    readBits(devAddr_ADXL345, ADXL345_RA_FIFO_CTL, ADXL345_FIFO_SAMPLES_BIT, ADXL345_FIFO_SAMPLES_LENGTH, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Set FIFO samples setting.
 * @param size New FIFO samples setting (impact depends on FIFO mode setting)
 * @see getFIFOSamples()
 * @see getFIFOMode()
 * @see ADXL345_RA_FIFO_CTL
 * @see ADXL345_FIFO_SAMPLES_BIT
 * @see ADXL345_FIFO_SAMPLES_LENGTH
 */
void setFIFOSamples(uint8_t size) {
    writeBits(devAddr_ADXL345, ADXL345_RA_FIFO_CTL, ADXL345_FIFO_SAMPLES_BIT, ADXL345_FIFO_SAMPLES_LENGTH, size);
}

// FIFO_STATUS register

/** Get FIFO trigger occurred status.
 * A 1 in the FIFO_TRIG bit corresponds to a trigger event occurring, and a 0
 * means that a FIFO trigger event has not occurred.
 * @return FIFO trigger occurred status
 * @see ADXL345_RA_FIFO_STATUS
 * @see ADXL345_FIFOSTAT_TRIGGER_BIT
 */
int getFIFOTriggerOccurred() {
    readBit(devAddr_ADXL345, ADXL345_RA_FIFO_STATUS, ADXL345_FIFOSTAT_TRIGGER_BIT, buffer_ADXL345, 0);
    return buffer_ADXL345[0];
}
/** Get FIFO length.
 * These bits report how many data values are stored in FIFO. Access to collect
 * the data from FIFO is provided through the DATAX, DATAY, and DATAZ registers.
 * FIFO reads must be done in burst or multiple-byte mode because each FIFO
 * level is cleared after any read (single- or multiple-byte) of FIFO. FIFO
 * stores a maximum of 32 entries, which equates to a maximum of 33 entries
 * available at any given time because an additional entry is available at the
 * output filter of the 
 * @return Current FIFO length
 * @see ADXL345_RA_FIFO_STATUS
 * @see ADXL345_FIFOSTAT_LENGTH_BIT
 * @see ADXL345_FIFOSTAT_LENGTH_LENGTH
 */
uint8_t getFIFOLength() {
  readBits(devAddr_ADXL345, ADXL345_RA_FIFO_STATUS, ADXL345_FIFOSTAT_LENGTH_BIT, ADXL345_FIFOSTAT_LENGTH_LENGTH, buffer_ADXL345, 0);
  return buffer_ADXL345[0];
}

int measure_ADXL345(float * ax_g, float * ay_g, float * az_g, uint8_t devAddr_ADXL345){
  // Measure each 10 bit axis
  int16_t ax, ay, az; 
  getAcceleration(&ax, &ay, &az);

  // Convert to gravity based on Data_format register setting (here +/-16 g)
  *ax_g = (float)ax*31.2*0.001;     // Convert to Gravity, 31.2 mG / LSb. 
  *ay_g = (float)ay*31.2*0.001;
  *az_g = (float)az*31.2*0.001;
  return 0; 
  }

int configure_ADXL345(uint8_t address) {
  uint8_t e;
  ADXL345(address);
  initialize_ADXL345();
  setOffset(0, 0, 0);											// Set all offsets to zero

  // Set to full 16 g range with full 13 bit resolution
  e= writeByte(devAddr_ADXL345, ADXL345_RA_DATA_FORMAT, ((0x00 |ADXL345_RANGE_16G) |
                                          ADXL345_FORMAT_FULL_RES_BIT)); 

  // Turn off FIFO, simply useB
  setFIFOMode_ADXL345(ADXL345_FIFO_MODE_BYPASS); 

  }







