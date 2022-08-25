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
 * date:	07/03/2014
 */

#include "l3g4200d.h"


uint8_t devAddr_L3G4200D;
uint8_t buffer_L3G4200D[6];

void L3G4200D(uint8_t address){
    devAddr_L3G4200D = address;
}

/** Specific address constructor.
 * @param address I2C address
 * @see L3G4200D_DEFAULT_ADDRESS
 * @see L3G4200D_ADDRESS


/** Power on and prepare for general usage.
 * All values are defaults except for the power on bit in CTRL_REG_1
 * @see L3G4200D_RA_CTRL_REG1
 * @see L3G4200D_RA_CTRL_REG2
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_RA_CTRL_REG4
 * @see L3G4200D_RA_CTRL_REG5
 */
uint8_t initialize_L3G4200D(void) {
    uint8_t e; 
    e= writeByte(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG1, 0b00001111);
    e= writeByte(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG2, 0b00000000);
    e = writeByte(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, 0b00000000);
    e = writeByte(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG4, 0b00000000);
    e= writeByte(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG5, 0b00000000);
    return e; 
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
int testConnection_L3G4200D(void) {
    return getDeviceID_L3G4200D() == 0b11010011;
}

// WHO_AM_I register, read-only

/** Get the Device ID.
 * The WHO_AM_I register holds the device's id
 * @return Device ID (should be 0b11010011, 109, 0x69)
 * @see L3G4200D_RA_WHO_AM_I
 */
uint8_t getDeviceID_L3G4200D(void) {
    readByte(devAddr_L3G4200D, L3G4200D_RA_WHO_AM_I, buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

// CTRL_REG1 register, r/w

/** Set the output data rate
 * @param rate The new data output rate (can be 100, 200, 400, or 800)
 * @see L3G4200D_RA_CTRL_REG1
 * @see L3G4200D_ODR_BIT
 * @see L3G4200D_ODR_LENGTH
 * @see L3G4200D_RATE_100
 * @see L3G4200D_RATE_200
 * @see L3G4200D_RATE_400
 * @see L3G4200D_RATE_800
 */
void setOutputDataRate(uint16_t rate) {
	uint8_t writeVal;

	if (rate == 100) {
		writeVal = L3G4200D_RATE_100;
	} else if (rate == 200) {
		writeVal = L3G4200D_RATE_200;
	} else if (rate == 400) {
		writeVal = L3G4200D_RATE_400;
	} else {
		writeVal = L3G4200D_RATE_800;
	}

	writeBits(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG1, L3G4200D_ODR_BIT,
		L3G4200D_ODR_LENGTH, writeVal); 
}

/** Get the current output data rate
 * @return Current data output rate
 * @see L3G4200D_RA_CTRL_REG1
 * @see L3G4200D_ODR_BIT
 * @see L3G4200D_ODR_LENGTH
 * @see L3G4200D_RATE_100
 * @see L3G4200D_RATE_200
 * @see L3G4200D_RATE_400
 * @see L3G4200D_RATE_800
 */
uint16_t getOutputDataRate(void) {
	uint8_t rate = buffer_L3G4200D[0];
        readBits(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG1, L3G4200D_ODR_BIT, 
		L3G4200D_ODR_LENGTH, buffer_L3G4200D, 0);
	

	if (rate == L3G4200D_RATE_100) {
		return 100;
	} else if (rate == L3G4200D_RATE_200) {
		return 200;
	} else if (rate == L3G4200D_RATE_400) {
		return 400;
	}
	return 800;
}

/** Set the bandwidth cut-off mode
 * @param mode The new bandwidth cut-off mode
 * @see L3G4200D_RA_CTRL_REG1
 * @see L3G4200D_BW_BIT
 * @see L3G4200D_BW_LENGTH
 * @see L3G4200D_BW_LOW
 * @see L3G4200D_BW_MED_LOW
 * @see L3G4200D_BW_MED_HIGH
 * @see L3G4200D_BW_HIGH
 */
void setBandwidthCutOffMode(uint8_t mode) {
	writeBits(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG1, L3G4200D_BW_BIT, 
		L3G4200D_BW_LENGTH, mode);
}

/** Get the current bandwidth cut-off mode
 * @return Current bandwidth cut off mode
 * @see L3G4200D_RA_CTRL_REG1
 * @see L3G4200D_BW_BIT
 * @see L3G4200D_BW_LENGTH
 * @see L3G4200D_BW_LOW
 * @see L3G4200D_BW_MED_LOW
 * @see L3G4200D_BW_MED_HIGH
 * @see L3G4200D_BW_HIGH
 */
uint8_t getBandwidthCutOffMode(void) {
	readBits(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG1, L3G4200D_BW_BIT, 
		L3G4200D_BW_LENGTH, buffer_L3G4200D, 0);
	return buffer_L3G4200D[0];
}

/** Gets the current bandwidth cutoff based on ODR and BW
 * @return Float value of the bandwidth cut off
 * @see L3G4200D_RA_CTRL_REG1
 * @see L3G4200D_ODR_BIT
 * @see L3G4200D_ODR_LENGTH
 * @see L3G4200D_RATE_100
 * @see L3G4200D_RATE_200
 * @see L3G4200D_RATE_400
 * @see L3G4200D_RATE_800
 * @see L3G4200D_BW_BIT
 * @see L3G4200D_BW_LENGTH
 * @see L3G4200D_BW_LOW
 * @see L3G4200D_BW_MED_LOW
 * @see L3G4200D_BW_MED_HIGH
 * @see L3G4200D_BW_HIGH
 */
float getBandwidthCutOff(void) {
	uint16_t dataRate = getOutputDataRate();
	uint8_t bandwidthMode = getBandwidthCutOffMode();

	if (dataRate == 100) {
		if (bandwidthMode == L3G4200D_BW_LOW) {
			return 12.5;
		} else {
			return 25.0;
		}
	} else if (dataRate == 200) {
		if (bandwidthMode == L3G4200D_BW_LOW) {
			return 12.5;
		} else if (bandwidthMode == L3G4200D_BW_MED_LOW) {
			return 25.0;
		} else if (bandwidthMode == L3G4200D_BW_MED_HIGH) {
			return 50.0;
		} else {
			return 70.0;
		}
	} else if (dataRate == 400) {
		if (bandwidthMode == L3G4200D_BW_LOW) {
			return 20.0;
		} else if (bandwidthMode == L3G4200D_BW_MED_LOW) {
			return 25.0;
		} else if (bandwidthMode == L3G4200D_BW_MED_HIGH) {
			return 50.0;
		} else {
			return 110.0;
		}
	} else {
		if (bandwidthMode == L3G4200D_BW_LOW) {
			return 30.0;
		} else if (bandwidthMode == L3G4200D_BW_MED_LOW) {
			return 35.0;
		} else if (bandwidthMode == L3G4200D_BW_MED_HIGH) {
			return 50.0;
		} else {
			return 110.0;
		}
	}
}

/** Set power on or off
 * @param enabled The new power setting (true for on, false for off)
 * @see L3G4200D_RA_CTRL_REG1
 * @see L3G4200D_PD_BIT
 */
void setPowerOn(int on) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG1, L3G4200D_PD_BIT, on);
}

/** Get the current power state
 * @return Powered on state (true for on, false for off)
 * @see L3G4200D_RA_CTRL_REG1
 * @see L3G4200D_PD_BIT
 */
int getPowerOn(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG1, L3G4200D_PD_BIT, buffer_L3G4200D, 0);
	return buffer_L3G4200D[0];
}

/** Enables or disables the ability to get Z data
 * @param enabled The new enabled state of the Z axis
 * @see L3G4200D_RA_CTRL_REG1
 * @see L3G4200D_ZEN_BIT
 */
void setZEnabled(int enabled) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG1, L3G4200D_ZEN_BIT, enabled);
}

/** Get whether Z axis data is enabled
 * @return True if the Z axis is enabled, false otherwise
 * @see L3G4200D_RA_CTRL_REG1
 * @see L3G4200D_ZEN_BIT
 */
int getZEnabled(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG1, L3G4200D_ZEN_BIT, buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Enables or disables the ability to get Y data
 * @param enabled The new enabled state of the Y axis
 * @see L3G4200D_RA_CTRL_REG1
 * @see L3G4200D_YEN_BIT
 */
void setYEnabled(int enabled) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG1, L3G4200D_YEN_BIT, enabled);
}

/** Get whether Y axis data is enabled
 * @return True if the Y axis is enabled, false otherwise
 * @see L3G4200D_RA_CTRL_REG1
 * @see L3G4200D_YEN_BIT
 */
int getYEnabled(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG1, L3G4200D_YEN_BIT, buffer_L3G4200D, 0);
	return buffer_L3G4200D[0];
}

/** Enables or disables the ability to get X data
 * @param enabled The new enabled state of the X axis
 * @see L3G4200D_RA_CTRL_REG1
 * @see L3G4200D_XEN_BIT
 */
void setXEnabled(int enabled) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG1, L3G4200D_XEN_BIT, enabled);
}

/** Get whether X axis data is enabled
 * @return True if the X axis is enabled, false otherwise
 * @see L3G4200D_RA_CTRL_REG1
 * @see L3G4200D_XEN_BIT
 */
int getXEnabled(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG1, L3G4200D_XEN_BIT, buffer_L3G4200D ,0);
	return buffer_L3G4200D[0];
}

// CTRL_REG2 register, r/w

/** Set the high pass mode
 * @param mode The new high pass mode
 * @see L3G4200D_RA_CTRL_REG2
 * @see L3G4200D_HPM_BIT
 * @see L3G4200D_HPM_LENGTH
 * @see L3G4200D_HPM_HRF
 * @see L3G4200D_HPM_REFERENCE
 * @see L3G4200D_HPM_NORMAL
 * @see L3G4200D_HPM_AUTORESET
 */
void setHighPassMode(uint8_t mode) {
	writeBits(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG2, L3G4200D_HPM_BIT, 
		L3G4200D_HPM_LENGTH, mode);
}

/** Get the high pass mode
 * @return High pass mode
 * @see L3G4200D_RA_CTRL_REG2
 * @see L3G4200D_HPM_BIT
 * @see L3G4200D_HPM_LENGTH
 * @see L3G4200D_HPM_HRF
 * @see L3G4200D_HPM_REFERENCE
 * @see L3G4200D_HPM_NORMAL
 * @see L3G4200D_HPM_AUTORESET
 */
uint8_t getHighPassMode(void) {
	readBits(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG2, L3G4200D_HPM_BIT, 
		L3G4200D_HPM_LENGTH, buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Set the high pass filter cut off frequency level (1 - 10)
 * @param level The new level for the hpcf, using one of the defined levels
 * @see L3G4200D_RA_CTRL_REG2
 * @see L3G4200D_HPCF_BIT
 * @see L3G4200D_HPCF_LENGTH
 * @see L3G4200D_HPCF1
 * @see L3G4200D_HPCF2
 * @see L3G4200D_HPCF3
 * @see L3G4200D_HPCF4
 * @see L3G4200D_HPCF5
 * @see L3G4200D_HPCF6
 * @see L3G4200D_HPCF7
 * @see L3G4200D_HPCF8
 * @see L3G4200D_HPCF9
 * @see L3G4200D_HPCF10
 */
void setHighPassFilterCutOffFrequencyLevel(uint8_t level) {
	writeBits(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG2, L3G4200D_HPCF_BIT, 
		L3G4200D_HPCF_LENGTH, level);
}

/** Get the high pass filter cut off frequency level (1 - 10)
 * @return High pass filter cut off frequency level
 * @see L3G4200D_RA_CTRL_REG2
 * @see L3G4200D_HPCF_BIT
 * @see L3G4200D_HPCF_LENGTH
 * @see L3G4200D_HPCF1
 * @see L3G4200D_HPCF2
 * @see L3G4200D_HPCF3
 * @see L3G4200D_HPCF4
 * @see L3G4200D_HPCF5
 * @see L3G4200D_HPCF6
 * @see L3G4200D_HPCF7
 * @see L3G4200D_HPCF8
 * @see L3G4200D_HPCF9
 * @see L3G4200D_HPCF10
 */
uint8_t getHighPassFilterCutOffFrequencyLevel(void) {
	readBits(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG2, L3G4200D_HPCF_BIT, 
		L3G4200D_HPCF_LENGTH, buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

// CTRL_REG3 register, r/w

/** Set the INT1 interrupt enabled state
 * @param enabled New enabled state for the INT1 interrupt
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_I1_INT1_BIT
 */
void setINT1InterruptEnabled(int enabled) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, L3G4200D_I1_INT1_BIT, 
		enabled);
}

/** Get the INT1 interrupt enabled state
 * @return True if the INT1 interrupt is enabled, false otherwise
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_I1_INT1_BIT
 */
int getINT1InterruptEnabled(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, L3G4200D_I1_INT1_BIT, 
		buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Set the INT1 boot status enabled state
 * @param enabled New enabled state for the INT1 boot status
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_I1_BOOT_BIT
 */
void setINT1BootStatusEnabled(int enabled) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, L3G4200D_I1_BOOT_BIT, 
		enabled);
}

/** Get the INT1 boot status enabled state
 * @return INT1 boot status status
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_I1_BOOT_BIT
 */
int getINT1BootStatusEnabled(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, L3G4200D_I1_BOOT_BIT, 
		buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Interrupts the active INT1 configuration
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_H_LACTIVE_BIT
 */
void interruptActiveINT1Config(void) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, L3G4200D_H_LACTIVE_BIT, 1);
}

/** Set output mode to push-pull or open-drain
 * @param mode New output mode
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_PP_OD_BIT
 * @see L3G4200D_PUSH_PULL
 * @see L3G4200D_OPEN_DRAIN
 */
void setOutputMode(int mode) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, L3G4200D_PP_OD_BIT, 
		mode);
}

/** Get whether mode is push-pull or open drain
 * @return Output mode (TRUE for push-pull, FALSE for open-drain)
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_PP_OD_BIT
 * @see L3G4200D_PUSH_PULL
 * @see L3G4200D_OPEN_DRAIN
 */
int getOutputMode(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, L3G4200D_PP_OD_BIT, 
		buffer_L3G4200D, 0);
	return buffer_L3G4200D[0];
}

/** Set data ready interrupt enabled state on INT2 pin
 * @param enabled New INT2 data ready interrupt enabled state
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_I2_DRDY_BIT
 */
void setINT2DataReadyEnabled(int enabled) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, L3G4200D_I2_DRDY_BIT, 
		enabled);
}

/** Get whether the data ready interrupt is enabled on the INT2 pin
 * @return True if the INT2 data ready interrupt is enabled, false otherwise
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_I2_DRDY_BIT
 */
int getINT2DataReadyEnabled(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, L3G4200D_I2_DRDY_BIT, 
		buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Set whether the INT2 FIFO watermark interrupt is enabled
 * The sensor contains a 32-slot FIFO buffer_L3G4200D for storing data so that it may be 
 * read later. If enabled, the sensor will generate an interrupt on the 
 * INT2/DRDY pin when the watermark has been reached. The watermark can be 
 * configured through the setFIFOWatermark function.
 * @param enabled New enabled state of the INT2 FIFO watermark
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_I2_WTM_BIT
 */
void setINT2FIFOWatermarkInterruptEnabled(int enabled) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, L3G4200D_I2_WTM_BIT, 
		enabled);
}

/** Get the INT2 FIFO watermark interrupt enabled state
 * @return true if the FIFO watermark is enabled, false otherwise
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_I2_WTM_BIT
 */ 
int getINT2FIFOWatermarkInterruptEnabled(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, L3G4200D_I2_WTM_BIT, 
		buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Set whether an interrupt is triggered on INT2 when the FIFO is overrun
 * @param enabled New FIFO overrun interrupt enabled state
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_I2_ORUN_BIT
 */
void setINT2FIFOOverrunInterruptEnabled(int enabled) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, L3G4200D_I2_ORUN_BIT, 
		enabled);
}

/** Get whether an interrupt is triggered on INT2 when the FIFO is overrun
 * @return True if the INT2 FIFO overrun interrupt is enabled, false otherwise
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_I2_ORUN_BIT
 */
int getINT2FIFOOverrunInterruptEnabled(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, L3G4200D_I2_ORUN_BIT,
		buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Set whether an interrupt is triggered on INT2 when the FIFO buffer_L3G4200D is empty
 * @param enabled New INT2 FIFO empty interrupt state
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_I2_EMPTY_BIT
 */
void setINT2FIFOEmptyInterruptEnabled(int enabled) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, L3G4200D_I2_EMPTY_BIT, 
		enabled);
}

/** Get whether the INT2 FIFO empty interrupt is enabled
 * @returns Trur if the INT2 FIFO empty interrupt is enabled, false otherwise
 * @see L3G4200D_RA_CTRL_REG3
 * @see L3G4200D_I2_EMPTY_BIT
 */
int getINT2FIFOEmptyInterruptEnabled(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG3, L3G4200D_I2_EMPTY_BIT, 
		buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

// CTRL_REG4 register, r/w

/** Set the Block Data Update (BDU) enabled state
 * @param enabled New BDU enabled state
 * @see L3G4200D_RA_CTRL_REG4
 * @see L3G4200D_BDU_BIT
 */
void setBlockDataUpdateEnabled(int enabled) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG4, L3G4200D_BDU_BIT, enabled);
}

/** Get the BDU enabled state
 * @return True if Block Data Update is enabled, false otherwise
 * @see L3G4200D_RA_CTRL_REG4
 * @see L3G4200D_BDU_BIT
 */
int getBlockDataUpdateEnabled(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG4, L3G4200D_BDU_BIT, buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Set the data endian modes
 * In Big Endian mode, the Most Significat Byte (MSB) is on the lower address, 
 * and the Least Significant Byte (LSB) is on the higher address. Little Endian 
 * mode reverses this order. Little Endian is the default mode.
 * @param endianness New endian mode
 * @see L3G4200D_RA_CTRL_REG4
 * @see L3G4200D_BLE_BIT
 * @see L3G4200D_BIG_ENDIAN
 * @see L3G4200D_LITTLE_ENDIAN
 */
void setEndianMode(int endianness) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG4, L3G4200D_BLE_BIT, 
		endianness);
}

/** Get the data endian mode
 * @return Current endian mode
 * @see L3G4200D_RA_CTRL_REG4
 * @see L3G4200D_BLE_BIT
 * @see L3G4200D_BIG_ENDIAN
 * @see L3G4200D_LITTLE_ENDIAN
 */
int getEndianMode(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG4, L3G4200D_BLE_BIT,
		buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Set the full scale of the data output (in dps)
 * @param scale The new scale of the data output (250, 500, 2000)
 * @see L3G4200D_RA_CTRL_REG4
 * @see L3G4200D_FS_BIT
 * @see L3G4200D_FS_LENGTH
 * @see L3G4200D_FS_250
 * @see L3G4200D_FS_500
 * @see L3G4200D_FS_2000
 */
void setFullScale(uint16_t scale) {
	uint8_t Bits;

	if (scale == 250) {
		Bits = L3G4200D_FS_250;
	} else if (scale == 500) {
                Bits = L3G4200D_FS_500;
	} else {
		Bits = L3G4200D_FS_2000;
	}

	writeBits(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG4, L3G4200D_FS_BIT, 
		L3G4200D_FS_LENGTH, Bits);
}

/** Get the current full scale of the output data (in dps)
 * @return Current scale of the output data
 * @see L3G4200D_RA_CTRL_REG4
 * @see L3G4200D_FS_BIT
 * @see L3G4200D_FS_LENGTH
 * @see L3G4200D_FS_250
 * @see L3G4200D_FS_500
 * @see L3G4200D_FS_2000
 */
uint16_t getFullScale(void) {
        uint8_t Bits = buffer_L3G4200D[0];
	readBits(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG4, 
		L3G4200D_FS_BIT, L3G4200D_FS_LENGTH, buffer_L3G4200D,0);
	

	if (Bits == L3G4200D_FS_250) {
		return 250;
	} else if (Bits == L3G4200D_FS_500) {
		return 500;
	} else {
		return 2000;
	}
}

/** Set the self test mode
 * @param mode New self test mode (Normal, 0, 1)
 * @see L3G4200D_RA_CTRL_REG4
 * @see L3G4200D_ST_BIT
 * @see L3G4200D_ST_LENGTH
 * @see L3G4200D_SELF_TEST_NORMAL
 * @see L3G4200D_SELF_TEST_0
 * @see L3G4200D_SELF_TEST_1
 */
void setSelfTestMode(uint8_t mode) {
	writeBits(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG4, L3G4200D_ST_BIT, 
		L3G4200D_ST_LENGTH, mode);
}

/** Get the current self test mode
 * @return Current self test mode
 * @see L3G4200D_RA_CTRL_REG4
 * @see L3G4200D_ST_BIT
 * @see L3G4200D_ST_LENGTH
 * @see L3G4200D_SELF_TEST_NORMAL
 * @see L3G4200D_SELF_TEST_0
 * @see L3G4200D_SELF_TEST_1
 */
uint8_t getSelfTestMode(void) {
	readBits(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG4, L3G4200D_ST_BIT, 
		L3G4200D_ST_LENGTH, buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Set the SPI mode
 * @param mode New SPI mode
 * @see L3G4200D_RA_CTRL_REG4
 * @see L3G4200D_SIM_BIT
 * @see L3G4200D_SPI_4_WIRE
 * @see L3G4200D_SPI_3_WIRE
 */
void setSPIMode_L3G4200D(int mode) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG4, L3G4200D_SIM_BIT, mode);
}

/** Get the SPI mode
 * @return Current SPI mode
 * @see L3G4200D_RA_CTRL_REG4
 * @see L3G4200D_SIM_BIT
 * @see L3G4200D_SPI_4_WIRE
 * @see L3G4200D_SPI_3_WIRE
 */
int getSPIMode_L3G4200D(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG4, L3G4200D_SIM_BIT, 
		buffer_L3G4200D,0);
 	return buffer_L3G4200D[0];
}

// CTRL_REG5 register, r/w

/** Reboots the FIFO memory content
 * @see L3G4200D_RA_CTRL_REG5
 * @see L3G4200D_BOOT_BIT
 */
void rebootMemoryContent(void) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG5, L3G4200D_BOOT_BIT, 1);
}

/** Set whether the FIFO buffer_L3G4200D is enabled
 * @param enabled New enabled state of the FIFO buffer_L3G4200D
 * @see L3G4200D_RA_CTRL_REG5
 * @see L3G4200D_FIFO_EN_BIT
 */
void setFIFOEnabled(int enabled) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG5, L3G4200D_FIFO_EN_BIT, 
		enabled);
}

/** Get whether the FIFO buffer_L3G4200D is enabled
 * @return True if the FIFO buffer_L3G4200D is enabled, false otherwise
 * @see L3G4200D_RA_CTRL_REG5
 * @see L3G4200D_FIFO_EN_BIT
 */
int getFIFOEnabled(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG5, L3G4200D_FIFO_EN_BIT, 
		buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Set the high pass filter enabled state
 * @param enabled New high pass filter enabled state
 * @see L3G4200D_RA_CTRL_REG5
 * @see L3G4200D_HPEN_BIT
 */
void setHighPassFilterEnabled(int enabled) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG5, L3G4200D_HPEN_BIT, 
		enabled);
}

/** Get whether the high pass filter is enabled
 * @return True if the high pass filter is enabled, false otherwise
 * @see L3G4200D_RA_CTRL_REG5
 * @see L3G4200D_HPEN_BIT
 */
int getHighPassFilterEnabled(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG5, L3G4200D_HPEN_BIT,
		buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Sets the filter mode to one of the four provided.
 * This function also uses the setHighPassFilterEnabled function in order to set
 * the mode. That function does not haved to be called in addition to this one. 
 * In addition to setting the filter for the data in the FIFO buffer_L3G4200D 
 * (controlled by the bits written to OUT_SEL), this function also sets the
 * filter used for interrupt generation (the bits written to INT1_SEL) to be the
 * same as the filter used for the FIFO buffer_L3G4200D.
 * @param filter New method to be used when filtering data
 * @see L3G4200D_RA_CTRL_REG5
 * @see L3G4200D_INT1_SEL_BIT
 * @see L3G4200D_INT1_SEL_LENGTH
 * @see L3G4200D_OUT_SEL_BIT
 * @see L3G4200D_OUT_SEL_LENGTH
 * @see L3G4200D_NON_HIGH_PASS
 * @see L3G4200D_HIGH_PASS
 * @see L3G4200D_LOW_PASS
 * @see L3G4200D_LOW_HIGH_PASS
 */
void setDataFilter(uint8_t filter) {
	if (filter == L3G4200D_HIGH_PASS || filter == L3G4200D_LOW_HIGH_PASS) {
		setHighPassFilterEnabled(1);
	} else {
		setHighPassFilterEnabled(0);
	}

	writeBits(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG5, L3G4200D_OUT_SEL_BIT, 
		L3G4200D_OUT_SEL_LENGTH, filter);
	writeBits(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG5, L3G4200D_INT1_SEL_BIT, 
		L3G4200D_INT1_SEL_LENGTH, filter);
}

/** Gets the data filter currently in use
 * @return Defined value that represents the filter in use
 * @see L3G4200D_RA_CTRL_REG5
 * @see L3G4200D_OUT_SEL_BIT
 * @see L3G4200D_OUT_SEL_LENGTH
 * @see L3G4200D_NON_HIGH_PASS
 * @see L3G4200D_HIGH_PASS
 * @see L3G4200D_LOW_PASS
 * @see L3G4200D_LOW_HIGH_PASS
 */
uint8_t getDataFilter(void) {
        uint8_t outBits = buffer_L3G4200D[0];
	readBits(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG5, L3G4200D_OUT_SEL_BIT, 
		L3G4200D_OUT_SEL_LENGTH, buffer_L3G4200D,0);

	if (outBits == L3G4200D_NON_HIGH_PASS || outBits == L3G4200D_HIGH_PASS) {
		return outBits;
	}

	if (getHighPassFilterEnabled()) {
		return L3G4200D_LOW_HIGH_PASS;
	} else {
		return L3G4200D_LOW_PASS;
	}
}

// REFERENCE/DATACAPTURE register, r/w

/** Set the reference value for interrupt generation
 * @param reference New reference value for interrupt generation
 * @see L3G4200D_RA_REFERENCE
 */
void setInterruptReference(uint8_t reference) {
	writeByte(devAddr_L3G4200D, L3G4200D_RA_REFERENCE, reference);
}

/** Get the 8-bit reference value for interrupt generation
 * @return 8-bit reference value for interrupt generation
 * @see L3G4200D_RA_REFERENCE
 */
uint8_t getInterruptReference(void) {
	readByte(devAddr_L3G4200D, L3G4200D_RA_REFERENCE, buffer_L3G4200D, 0);
	return buffer_L3G4200D[0];
}

// OUT_TEMP register, read-only

/** Gets the current temperature reading from the sensor
 * @return Current temperature
 * @see L3G4200D_RA_OUT_TEMP
 */
uint8_t getTemperature(void) {
	readByte(devAddr_L3G4200D, L3G4200D_RA_OUT_TEMP, buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

// STATUS register, read-only

/** Get whether new data overwrote the last set of data before it was read
 * @return True if the last set of data was overwritten before being read, false
 * otherwise
 * @see L3G4200D_RA_STATUS
 * @see L3G4200D_ZYXOR_BIT
 */
int getXYZOverrun(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_STATUS, L3G4200D_ZYXOR_BIT, 
		buffer_L3G4200D, 0);
	return buffer_L3G4200D[0];
}

/** Get whether new Z data overwrote the last set of data before it was read
 * @return True if the last set of Z data was overwritten before being read,
 * false otherwise
 * @see L3G4200D_RA_STATUS
 * @see L3G4200D_ZOR_BIT
 */
int getZOverrun(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_STATUS, L3G4200D_ZOR_BIT, 
		buffer_L3G4200D, 0);
	return buffer_L3G4200D[0];
}

/** Get whether new Y data overwrote the last set of data before it was read
 * @return True if the last set of Y data was overwritten before being read, 
 * false otherwise
 * @see L3G4200D_RA_STATUS
 * @see L3G4200D_YOR_BIT
 */
int getYOverrun(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_STATUS, L3G4200D_YOR_BIT, 
		buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Get whether new X data overwrote the last set of data before it was read
 * @return True if the last set of X data was overwritten before being read, 
 * false otherwise
 * @see L3G4200D_RA_STATUS
 * @see L3G4200D_XOR_BIT
 */
int getXOverrun(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_STATUS, L3G4200D_XOR_BIT, 
		buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Get whether there is new data avaialable
 * @return True if there is new data available, false otherwise
 * @see L3G4200D_RA_STATUS
 * @see L3G4200D_ZYXDA_BIT
 */
int getXYZDataAvailable(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_STATUS, L3G4200D_ZYXDA_BIT, 
		buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Get whether there is new Z data avaialable
 * @return True if there is new Z data available, false otherwise
 * @see L3G4200D_RA_STATUS
 * @see L3G4200D_ZDA_BIT
 */
int getZDataAvailable(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_STATUS, L3G4200D_ZDA_BIT, 
		buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Get whether there is new Y data avaialable
 * @return True if there is new Y data available, false otherwise
 * @see L3G4200D_RA_STATUS
 * @see L3G4200D_YDA_BIT
 */
int getYDataAvailable(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_STATUS, L3G4200D_YDA_BIT, 
		buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Get whether there is new X data avaialable
 * @return True if there is new X data available, false otherwise
 * @see L3G4200D_RA_STATUS
 * @see L3G4200D_XDA_BIT
 */
int getXDataAvailable(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_STATUS, L3G4200D_XDA_BIT, 
		buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

// OUT_* registers, read-only

/** Get the angular velocity for all 3 axes
 * Due to the fact that this device supports two difference Endian modes, both 
 * must be accounted for when reading data. In Little Endian mode, the first 
 * byte (lowest address) is the least significant and in Big Endian mode the 
 * first byte is the most significant.
 * @param x 16-bit integer container for the X-axis angular velocity
 * @param y 16-bit integer container for the Y-axis angular velocity
 * @param z 16-bit integer container for the Z-axis angular velocity
 */
void getAngularVelocity(int16_t* x, int16_t* y, int16_t* z) {
	*x = getAngularVelocityX();
	*y = getAngularVelocityY();
	*z = getAngularVelocityZ();
}

/** Get the angular velocity about the X-axis
 * @return Angular velocity about the X-axis
 * @see L3G4200D_RA_OUT_X_L
 * @see L3G4200D_RA_OUT_X_H
 */
int16_t getAngularVelocityX(void) {
	readBytes(devAddr_L3G4200D, L3G4200D_RA_OUT_X_L, 2, buffer_L3G4200D,0);
	if (getEndianMode() == L3G4200D_BIG_ENDIAN) {
		return (((int16_t)buffer_L3G4200D[1]) << 8) | buffer_L3G4200D[0];
	} else {
		return (((int16_t)buffer_L3G4200D[0]) << 8) | buffer_L3G4200D[1];
	}
}

/** Get the angular velocity about the Y-axis
 * @return Angular velocity about the Y-axis
 * @see L3G4200D_RA_OUT_Y_L
 * @see L3G4200D_RA_OUT_Y_H
 */
int16_t getAngularVelocityY(void) {
	readBytes(devAddr_L3G4200D, L3G4200D_RA_OUT_Y_L, 2, buffer_L3G4200D,0);
	if (getEndianMode() == L3G4200D_BIG_ENDIAN) {
		return (((int16_t)buffer_L3G4200D[1]) << 8) | buffer_L3G4200D[0];
	} else {
		return (((int16_t)buffer_L3G4200D[0]) << 8) | buffer_L3G4200D[1];
	}
}

/** Get the angular velocity about the Z-axis
 * @return Angular velocity about the Z-axis
 * @see L3G4200D_RA_OUT_Z_L
 * @see L3G4200D_RA_OUT_Z_H
 */
int16_t getAngularVelocityZ(void) {
	readBytes(devAddr_L3G4200D, L3G4200D_RA_OUT_Z_L, 2, buffer_L3G4200D,0);
	if (getEndianMode() == L3G4200D_BIG_ENDIAN) {
		return (((int16_t)buffer_L3G4200D[1]) << 8) | buffer_L3G4200D[0];
	} else {
		return (((int16_t)buffer_L3G4200D[0]) << 8) | buffer_L3G4200D[1];
	}
}

// FIFO_CTRL register, r/w

/** Set the FIFO mode to one of the defined modes
 * @param mode New FIFO mode
 * @see L3G4200D_RA_FIFO_CTRL
 * @see L3G4200D_FIFO_MODE_BIT
 * @see L3G4200D_FIFO_MODE_LENGTH
 * @see L3G4200D_FM_BYPASS
 * @see L3G4200D_FM_FIFO
 * @see L3G4200D_FM_STREAM
 * @see L3G4200D_FM_STREAM_FIFO
 * @see L3G4200D_FM_BYPASS_STREAM
 */
void setFIFOMode_L3G4200D(uint8_t mode) {
	writeBits(devAddr_L3G4200D, L3G4200D_RA_FIFO_CTRL, L3G4200D_FIFO_MODE_BIT, 
		L3G4200D_FIFO_MODE_LENGTH, mode);
}

/** Get the FIFO mode to one of the defined modes
 * @return Current FIFO mode
 * @see L3G4200D_RA_FIFO_CTRL
 * @see L3G4200D_FIFO_MODE_BIT
 * @see L3G4200D_FIFO_MODE_LENGTH
 * @see L3G4200D_FM_BYPASS
 * @see L3G4200D_FM_FIFO
 * @see L3G4200D_FM_STREAM
 * @see L3G4200D_FM_STREAM_FIFO
 * @see L3G4200D_FM_BYPASS_STREAM
 */
uint8_t getFIFOMode_L3G4200D(void) {
	readBits(devAddr_L3G4200D, L3G4200D_RA_FIFO_CTRL, 
		L3G4200D_FIFO_MODE_BIT, L3G4200D_FIFO_MODE_LENGTH, buffer_L3G4200D,0);
	return buffer_L3G4200D[0];
}

/** Set the FIFO watermark threshold
 * @param wtm New FIFO watermark threshold
 * @see L3G4200D_RA_FIFO_CTRL
 * @see L3G4200D_FIFO_WTM_BIT
 * @see L3G4200D_FIFO_WTM_LENGTH
 */
void setFIFOThreshold(uint8_t wtm) {
    writeBits(devAddr_L3G4200D, L3G4200D_RA_FIFO_CTRL, L3G4200D_FIFO_WTM_BIT, 
        L3G4200D_FIFO_WTM_LENGTH, wtm);
}

/** Get the FIFO watermark threshold
 * @return FIFO watermark threshold
 * @see L3G4200D_RA_FIFO_CTRL
 * @see L3G4200D_FIFO_WTM_BIT
 * @see L3G4200D_FIFO_WTM_LENGTH
 */
uint8_t getFIFOThreshold(void) {
    readBits(devAddr_L3G4200D, L3G4200D_RA_FIFO_CTRL, L3G4200D_FIFO_WTM_BIT,
        L3G4200D_FIFO_WTM_LENGTH, buffer_L3G4200D,0);
    return buffer_L3G4200D[0];
}

// FIFO_SRC register, read-only

/** Get whether the number of data sets in the FIFO buffer_L3G4200D is less than the 
 * watermark
 * @return True if the number of data sets in the FIFO buffer_L3G4200D is more than or 
 * equal to the watermark, false otherwise.
 * @see L3G4200D_RA_FIFO_SRC
 * @see L3G4200D_FIFO_STATUS_BIT
 */
int getFIFOAtWatermark(void) {
   	readBit(devAddr_L3G4200D, L3G4200D_RA_FIFO_SRC, L3G4200D_FIFO_STATUS_BIT, 
        buffer_L3G4200D,0);
   	return buffer_L3G4200D[0];
}

/** Get whether the FIFO buffer_L3G4200D is full
 * @return True if the FIFO buffer_L3G4200D is full, false otherwise
 * @see L3G4200D_RA_FIFO_SRC
 * @see L3G4200D_FIFO_OVRN_BIT
 */
int getFIFOOverrun(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_FIFO_SRC, 
        L3G4200D_FIFO_OVRN_BIT, buffer_L3G4200D,0);
    return buffer_L3G4200D[0];
}

/** Get whether the FIFO buffer_L3G4200D is empty
 * @return True if the FIFO buffer_L3G4200D is empty, false otherwise
 * @see L3G4200D_RA_FIFO_SRC
 * @see L3G4200D_FIFO_EMPTY_BIT
 */
int getFIFOEmpty(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_FIFO_SRC,
        L3G4200D_FIFO_EMPTY_BIT, buffer_L3G4200D,0);
    return buffer_L3G4200D[0];
}

/** Get the number of filled FIFO buffer_L3G4200D slots
 * @return Number of filled slots in the FIFO buffer_L3G4200D
 * @see L3G4200D_RA_FIFO_SRC
 * @see L3G4200D_FIFO_FSS_BIT
 * @see L3G4200D_FIFO_FSS_LENGTH
 */ 
uint8_t getFIFOStoredDataLevel(void) {
    readBits(devAddr_L3G4200D, L3G4200D_RA_FIFO_SRC, 
        L3G4200D_FIFO_FSS_BIT, L3G4200D_FIFO_FSS_LENGTH, buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

// INT1_CFG register, r/w

/** Set the combination mode for interrupt events
 * @param combination New combination mode for interrupt events. 
 * L3G4200D_INT1_OR for OR and L3G4200D_INT1_AND for AND
 * @see L3G4200D_RA_INT1_CFG
 * @see L3G4200D_INT1_AND_OR_BIT
 * @see L3G4200D_INT1_OR
 * @see L3G4200D_INT1_AND
 */
void setInterruptCombination(int combination) {
    writeBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_INT1_AND_OR_BIT,
        combination);
}

/** Get the combination mode for interrupt events
 * @return Combination mode for interrupt events. L3G4200D_INT1_OR for OR and 
 * L3G4200D_INT1_AND for AND
 * @see L3G4200D_RA_INT1_CFG
 * @see L3G4200D_INT1_AND_OR_BIT
 * @see L3G4200D_INT1_OR
 * @see L3G4200D_INT1_AND
 */
int getInterruptCombination(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_INT1_AND_OR_BIT,
        buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

/** Set whether an interrupt request is latched
 * This bit is cleared when the INT1_SRC register is read
 * @param latched New status of the latched request
 * @see L3G4200D_RA_INT1_CFG
 * @see L3G4200D_INT1_LIR_BIT
 */
void setInterruptRequestLatched(int latched) {
    writeBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_INT1_LIR_BIT, latched);
}

/** Get whether an interrupt request is latched
 * @return True if an interrupt request is latched, false otherwise
 * @see L3G4200D_RA_INT1_CFG
 * @see L3G4200D_INT1_LIR_BIT
 */
int getInterruptRequestLatched(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_INT1_LIR_BIT, 
        buffer_L3G4200D, 0); 
    return buffer_L3G4200D[0];
}

/** Set whether the interrupt for Z high is enabled
 * @param enabled New enabled state for Z high interrupt.
 * @see L3G4200D_INT1_CFG
 * @see L3G4200D_ZHIE_BIT
 */
void setZHighInterruptEnabled(int enabled) {
    writeBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_ZHIE_BIT, enabled);
}

/** Get whether the interrupt for Z high is enabled
 * @return True if the interrupt for Z high is enabled, false otherwise 
 * @see L3G4200D_INT1_CFG
 * @see L3G4200D_ZHIE_BIT
 */
int getZHighInterruptEnabled(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_ZHIE_BIT, 
        buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

/** Set whether the interrupt for Z low is enabled
 * @param enabled New enabled state for Z low interrupt.
 * @see L3G4200D_INT1_CFG
 * @see L3G4200D_ZLIE_BIT
 */
void setZLowInterruptEnabled(int enabled) {
    writeBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_ZLIE_BIT, enabled);
}

/** Get whether the interrupt for Z low is enabled
 * @return True if the interrupt for Z low is enabled, false otherwise
 * @see L3G4200D_INT1_CFG
 * @see L3G4200D_ZLIE_BIT
 */
int getZLowInterruptEnabled(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_ZLIE_BIT, 
        buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

/** Set whether the interrupt for Y high is enabled
 * @param enabled New enabled state for Y high interrupt.
 * @see L3G4200D_INT1_CFG
 * @see L3G4200D_YHIE_BIT
 */
void setYHighInterruptEnabled(int enabled) {
    writeBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_YHIE_BIT, enabled);
}

/** Get whether the interrupt for Y high is enabled
 * @return True if the interrupt for Y high is enabled, false otherwise
 * @see L3G4200D_INT1_CFG
 * @see L3G4200D_YHIE_BIT
 */
int getYHighInterruptEnabled(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_YHIE_BIT, 
        buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

/** Set whether the interrupt for Y low is enabled
 * @param enabled New enabled state for Y low interrupt.
 * @see L3G4200D_INT1_CFG
 * @see L3G4200D_YLIE_BIT
 */
void setYLowInterruptEnabled(int enabled) {
    writeBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_YLIE_BIT, enabled);
}

/** Get whether the interrupt for Y low is enabled
 * @return True if the interrupt for Y low is enabled, false otherwise 
 * @see L3G4200D_INT1_CFG
 * @see L3G4200D_YLIE_BIT
 */
int getYLowInterruptEnabled(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_YLIE_BIT, 
        buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

/** Set whether the interrupt for X high is enabled
 * @param enabled New enabled state for X high interrupt.
 * @see L3G4200D_INT1_CFG
 * @see L3G4200D_XHIE_BIT
 */
void setXHighInterruptEnabled(int enabled) {
    writeBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_XHIE_BIT, enabled);
}

/** Get whether the interrupt for X high is enabled
 * @return True if the interrupt for X high is enabled, false otherwise
 * @see L3G4200D_INT1_CFG
 * @see L3G4200D_XHIE_BIT
 */
int getXHighInterruptEnabled(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_XHIE_BIT, 
        buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

/** Set whether the interrupt for X low is enabled
 * @param enabled New enabled state for X low interrupt.
 * @see L3G4200D_INT1_CFG
 * @see L3G4200D_XLIE_BIT
 */
void setXLowInterruptEnabled(int enabled) {
    writeBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_XLIE_BIT, enabled);
}

/** Get whether the interrupt for X low is enabled
 * @return True if the interrupt for X low is enabled, false otherwise
 * @see L3G4200D_INT1_CFG
 * @see L3G4200D_XLIE_BIT
 */
int getXLowInterruptEnabled(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_CFG, L3G4200D_XLIE_BIT, 
        buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

// INT1_SRC register, read-only

/** Get whether an interrupt has been generated
 * @return True if one or more interrupts has been generated, false otherwise
 * @see L3G4200D_RA_INT1_SRC
 * @see L3G4200D_INT1_IA_BIT
 */
int getInterruptActive(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_SRC, L3G4200D_INT1_IA_BIT,
        buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

/** Get whether a Z high event has occurred
 * @return True if a Z high event has occurred, false otherwise
 * @see L3G4200D_RA_INT1_SRC
 * @see L3G4200D_INT1_ZH_BIT
 */
int getZHigh(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_SRC, L3G4200D_INT1_ZH_BIT,
        buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

/** Get whether a Z low event has occurred
 * @return True if a Z low event has occurred, false otherwise
 * @see L3G4200D_RA_INT1_SRC
 * @see L3G4200D_INT1_ZL_BIT
 */
int getZLow(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_SRC, L3G4200D_INT1_ZL_BIT,
        buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

/** Get whether a Y high event has occurred
 * @return True if a Y high event has occurred, false otherwise
 * @see L3G4200D_RA_INT1_SRC
 * @see L3G4200D_INT1_YH_BIT
 */
int getYHigh(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_SRC, L3G4200D_INT1_YH_BIT,
        buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

/** Get whether a Y low event has occurred
 * @return True if a Y low event has occurred, false otherwise
 * @see L3G4200D_RA_INT1_SRC
 * @see L3G4200D_INT1_YL_BIT
 */
int getYLow(void) {
   	readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_SRC, L3G4200D_INT1_YL_BIT,
        buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

/** Get whether a X high event has occurred
 * @return True if a X high event has occurred, false otherwise
 * @see L3G4200D_RA_INT1_SRC
 * @see L3G4200D_INT1_XH_BIT
 */
int getXHigh(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_SRC, L3G4200D_INT1_XH_BIT,
        buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

/** Get whether a X low event has occurred
 * @return True if a X low event has occurred, false otherwise
 * @see L3G4200D_RA_INT1_SRC
 * @see L3G4200D_INT1_XL_BIT
 */
int getXLow(void) {
    readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_SRC, L3G4200D_INT1_XL_BIT,
        buffer_L3G4200D, 0);
    return buffer_L3G4200D[0];
}

// INT1_THS_* registers, r/w

/** Set the threshold for a high interrupt on the X axis
 * @param threshold New threshold for a high interrupt on the X axis
 * @see L3G4200D_INT1_THS_XH
 */
void setXHighThreshold(uint8_t threshold) {
    writeByte(devAddr_L3G4200D, L3G4200D_RA_INT1_THS_XH, threshold);
}

/** Retrieve the threshold for a high interrupt on the X axis
 * @return X high interrupt threshold
 * @see L3G4200D_INT1_THS_XH
 */
uint8_t getXHighThreshold(void) {
	readByte(devAddr_L3G4200D, L3G4200D_RA_INT1_THS_XH, buffer_L3G4200D, 0);
	return buffer_L3G4200D[0];
}

/** Set the threshold for a low interrupt on the X axis
 * @param threshold New threshold for a low interrupt on the X axis
 * @see L3G4200D_INT1_THS_XL
 */
void setXLowThreshold(uint8_t threshold) {
    writeByte(devAddr_L3G4200D, L3G4200D_RA_INT1_THS_XL, threshold);
}

/** Retrieve the threshold for a low interrupt on the X axis
 * @return X low interrupt threshold
 * @see L3G4200D_INT1_THS_XL
 */
uint8_t getXLowThreshold(void) {
	readByte(devAddr_L3G4200D, L3G4200D_RA_INT1_THS_XL, buffer_L3G4200D, 0);
	return buffer_L3G4200D[0];
}

/** Set the threshold for a high interrupt on the Y axis
 * @param threshold New threshold for a high interrupt on the Y axis
 * @see L3G4200D_INT1_THS_YH
 */
void setYHighThreshold(uint8_t threshold) {
    writeByte(devAddr_L3G4200D, L3G4200D_RA_INT1_THS_YH, threshold);
}

/** Retrieve the threshold for a high interrupt on the Y axis
 * @return Y high interrupt threshold
 * @see L3G4200D_INT1_THS_YH
 */
uint8_t getYHighThreshold(void) {
	readByte(devAddr_L3G4200D, L3G4200D_RA_INT1_THS_YH, buffer_L3G4200D, 0);
	return buffer_L3G4200D[0];
}

/** Set the threshold for a low interrupt on the Y axis
 * @param threshold New threshold for a low interrupt on the Y axis
 * @see L3G4200D_INT1_THS_YL
 */
void setYLowThreshold(uint8_t threshold) {
    writeByte(devAddr_L3G4200D, L3G4200D_RA_INT1_THS_YL, threshold);
}

/** Retrieve the threshold for a low interrupt on the Y axis
 * @return Y low interrupt threshold
 * @see L3G4200D_INT1_THS_YL
 */
uint8_t getYLowThreshold(void) {
	readByte(devAddr_L3G4200D, L3G4200D_RA_INT1_THS_YL, buffer_L3G4200D, 0);
	return buffer_L3G4200D[0];
}

/** Set the threshold for a high interrupt on the Z axis
 * @param threshold New threshold for a high interrupt on the Z axis
 * @see L3G4200D_INT1_THS_ZH
 */
void setZHighThreshold(uint8_t threshold) {
    writeByte(devAddr_L3G4200D, L3G4200D_RA_INT1_THS_ZH, threshold);
}

/** Retrieve the threshold for a high interrupt on the Z axis
 * @return Z high interrupt threshold
 * @see L3G4200D_INT1_THS_ZH
 */
uint8_t getZHighThreshold(void) {
	readByte(devAddr_L3G4200D, L3G4200D_RA_INT1_THS_ZH, buffer_L3G4200D, 0);
	return buffer_L3G4200D[0];
}

/** Set the threshold for a low interrupt on the Z axis
 * @param threshold New threshold for a low interrupt on the Z axis
 * @see L3G4200D_RA_INT1_THS_ZL
 */
void setZLowThreshold(uint8_t threshold) {
    writeByte(devAddr_L3G4200D, L3G4200D_RA_INT1_THS_ZL, threshold);
}

/** Retrieve the threshold for a low interrupt on the Z axis
 * @return Z low interrupt threshold
 * @see L3G4200D_INT1_THS_ZL
 */
uint8_t getZLowThreshold(void) {
	readByte(devAddr_L3G4200D, L3G4200D_RA_INT1_THS_ZL, buffer_L3G4200D, 0);
	return buffer_L3G4200D[0];
}

// INT1_DURATION register, r/w

/* Set the minimum duration for an interrupt event to be recognized
 * This depends on the chosen output data rate
 * @param duration New duration necessary for an interrupt event to be 
 * recognized
 * @see L3G4200D_RA_INT1_DURATION
 * @see L3G4200D_INT1_DUR_BIT
 * @see L3G4200D_INT1_DUR_LENGTH
 */
void setDuration(uint8_t duration) {
	writeBits(devAddr_L3G4200D, L3G4200D_RA_INT1_DURATION, L3G4200D_INT1_DUR_BIT,
		L3G4200D_INT1_DUR_LENGTH, duration);
}

/** Get the minimum duration for an interrupt event to be recognized
 * @return Duration necessary for an interrupt event to be recognized
 * @see L3G4200D_RA_INT1_DURATION
 * @see L3G4200D_INT1_DUR_BIT
 * @see L3G4200D_INT1_DUR_LENGTH
 */
uint8_t getDuration(void) {
	readBits(devAddr_L3G4200D, L3G4200D_RA_INT1_DURATION, 
		L3G4200D_INT1_DUR_BIT, L3G4200D_INT1_DUR_LENGTH, buffer_L3G4200D, 0);
	return buffer_L3G4200D[0];
}

/** Set whether the interrupt wait feature is enabled
 * If false, the interrupt falls immediately if signal crosses the selected 
 * threshold. Otherwise, if signal crosses the selected threshold, the interrupt
 * falls only after the duration has counted number of samples at the selected 
 * data rate, written into the duration counter register.
 * @param enabled New enabled state of the interrupt wait
 * @see L3G4200D_RA_INT1_DURATION
 * @see L3G4200D_INT1_WAIT_BIT
 */
void setWaitEnabled(int enabled) {
	writeBit(devAddr_L3G4200D, L3G4200D_RA_INT1_DURATION, L3G4200D_INT1_WAIT_BIT,
		enabled);
}

/** Get whether the interrupt wait feature is enabled
 * @return True if the wait feature is enabled, false otherwise
 * @see L3G4200D_RA_INT1_DURATION
 * @see L3G4200D_INT1_WAIT_BIT
 */
int getWaitEnabled(void) {
	readBit(devAddr_L3G4200D, L3G4200D_RA_INT1_DURATION, 
		L3G4200D_INT1_WAIT_BIT, buffer_L3G4200D, 0);
	return buffer_L3G4200D[0];
}

/*************************************************************************************
 * Function:        void measureL3G4200D(uint8_t dev_addr, float * gyro_TEMP_output, float * gyro_output_X, 
 *                  float * gyro_output_Y, float * gyro_output_Z)
 *
 * PreCondition:    I2C has been enabled and configured 
 *
 * Input:           Parameters:
 *                    - Address of the l3g4200d you want to talk to 
 *                    - pointer Float for temp output
 *                    - float pointers for 3 axis gyro data
 *
 * Output:          Any error code from i2c.c
 *
 * Side Effects:    Measures l3g4200d axis
 *
 * Overview:        Configures the gyro to collect data (see comments on configuration)
 *                  Measures on chip temp and 3 axis gyroscope 
 *
 * Note:            See page 4 of the datasheet for info on low and high pass filtering
 *                  
 *                  
 *************************************************************************************/
uint8_t measureL3G4200D(uint8_t dev_addr, int16_t * gyro_TEMP_output, float * gyro_output_X, 
float * gyro_output_Y, float * gyro_output_Z )
{
  uint8_t status_reg_out; 

/********************************************************************************
*
* Collect 2 byte data from measurement registers and form into an integer for 
* later conversion to floating point degree per second measurement 
*
*******************************************************************************/
  uint8_t e =0x00; 
  uint8_t w_reg4[] = {L3G4200D_RA_CTRL_REG4, 0xA0};
  uint8_t w_reg5[] = {L3G4200D_RA_CTRL_REG5, 0x40};
  uint8_t w_reg1[] = {L3G4200D_RA_CTRL_REG1, 0x0F};
  uint8_t start_read = 0xA6; 
  uint8_t Read_Byte_Array[0x08];
  
  /*
  * Control Register 4 setup: 1010000
  * BIT7 = 1: BDU, Block update on 
  * BIT6 = 0: BLE, LSB at low address
  * BIT5+4 = 10: FS1 & FS0, Full scale is 2000 dps
  * BIT 3 --
  * BIT2+1 = 0: ST1 ST0, Self test disable 
  * BIT0 = 0: Protocol selection, i2c
  */
  //e =I2C_Write(&w_reg4, 0x02, dev_addr);

  /*
  * Control Register 4 setup: 1000000
  * BIT7 = 1: BOOT, Reboot memory content
  * BIT6 = 0: FIFO_EN, FIFO disabled
  * BIT5 --
  * BIT4 = 0: HPen, High pass filter disabled
  * BIT3+2 = 00: INT1 selection
  * BIT1+0 = 0: OUT_Sel1& OUT_Sel0,
  */
  //e = I2C_Write(&w_reg5, 0x02, dev_addr);

   /*
  * Control Register 4 setup: 00001111
  * BIT7+6 = 00: DR1 - DR0: Output data rate selection 100 Hz
  * BIT5+4 = 00: BW0- BW1: Bandwidth selection cut off of 12.5
  * BIT3 = 1: PD: Power down mode enable
  * BIT2 = 1: Zen,Y enabled
  * BIT1 = 1: Xen, X enabled
  * BIT0 = 1: Yen, Y enabled 
  */
 // e = I2C_Write(&w_reg1, 0x02, dev_addr);
  
  /*
  * Write to sub address where temp data is stored with auto increment selected 
  * with the MSb = 1
  */
  e= I2C_Write(&start_read, 0x01, dev_addr); 

  // Read requested data 
  e = I2C_Read(&Read_Byte_Array ,0x08, dev_addr);

  // Processing of Data
  *gyro_TEMP_output = (int16_t)(Read_Byte_Array[0]);
  status_reg_out = (Read_Byte_Array[1]);
  *gyro_output_X =  0.06103515625*((float)((int16_t)Read_Byte_Array[3] << 8 | (int16_t)Read_Byte_Array[2]));
  *gyro_output_Y =  0.06103515625*((float)((int16_t)Read_Byte_Array[5] << 8 | (int16_t)Read_Byte_Array[4]));
  *gyro_output_Z =  0.06103515625*((float)((int16_t)Read_Byte_Array[7] << 8 | (int16_t)Read_Byte_Array[6]));
  return e; 
}

uint8_t configureL3G4200D(uint8_t address )
  {
  uint8_t e; 
  uint8_t w_reg4[] = {L3G4200D_RA_CTRL_REG4, 0xA0};
  uint8_t w_reg5[] = {L3G4200D_RA_CTRL_REG5, 0x40};
  uint8_t w_reg1[] = {L3G4200D_RA_CTRL_REG1, 0x0F};
  e= I2C_Write(&w_reg4, 0x02, address);
  e= I2C_Write(&w_reg5, 0x02, address);
  e= I2C_Write(&w_reg1, 0x02, address);
/*************************************************************************************
* After a 5 ms powerup where the device reads its own registers follow this write 
* sequence to configure the device	
*		1. Write CTRL_REG2
*		2. Write CTRL_REG3
* 	3. Write CTRL_REG4
* 	4. Write CTRL_REG6
* 	5. Write Reference
* 	6. Write INT1_THS
* 	7. Write INT1_DUR
*       8. Write INT1_CFG
*	9. Write CTRL_REG5
*	10. Write CTRL_REG1
*************************************************************************************/
	
  /* 
  * CTRL_REG2:
  * configure high pass filter mode, and highpass cutoff at 0.1 Hz
  * assuming 100 Hz sample rate 
  */
  //L3G4200D(address);
  //initialize_L3G4200D();
  //setHighPassMode(L3G4200D_HPM_NORMAL);
  //setHighPassFilterCutOffFrequencyLevel(L3G4200D_HPCF7);
	
  /* 
  * CTRL_REG3:
  * Configure chip level interrupts, ignore this for now.
  */

  /* 
  * CTRL_REG4:
  * Configure to big endian, full scale of 2000 dps and no self test  
  */
  //setEndianMode(L3G4200D_BIG_ENDIAN);
  //setFullScale(L3G4200D_FS_2000); 
  //setSelfTestMode(L3G4200D_SELF_TEST_0); 

  /* 
  * CTRL_REG5:
  * Configure to enable high pass filtering  
  */
  //setHighPassFilterEnabled(L3G4200D_HPEN_BIT);

  /* 
  * CTRL_REG1: Configured in initialize 
  * LOW_PASS_FILTER_SETTINGD = DR100Hz_BW12p5Hz(0), 
  * Power_Down_Mode(1), XYZ_ENABLE_MODE(111) - 0b00001111 = 0x0F
  */
  //writeByte(devAddr_L3G4200D, L3G4200D_RA_CTRL_REG1, 0b00001111);
    return e; 
  }

