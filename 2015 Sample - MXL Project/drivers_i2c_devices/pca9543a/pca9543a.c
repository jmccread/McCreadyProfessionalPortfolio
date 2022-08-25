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
 * date:	08/12/2014
 */

/******************************************************************************
*
* bus_isolation.c
* Manages the various modes of isolation needed to allow Cerebro to function 
*
******************************************************************************/

/******************************************************************************
*
* Includes 
*
******************************************************************************/
#include "pca9543a.h"
#include "main.h"
/******************************************************************************
*
* Definitions   
*
******************************************************************************/

 /*************************************************************************************
 * Function:        uint8_t tca9548_channel_select(uint8_t channels, uint8_t address)
 *
 * PreCondition:    I2C has been properly configured with I2C_INIT_MASTER and I2c pins
 *                  have been configured 
 *
 * Input:           Parameters: 
 *                    - uint8_t channels:
 *                      Bit0 = P0 polarity inverted  
 *                      ~Bit0 = P0 enabled as output 
 *                      Bit1 = P1 enabled as input 
 *                      ~Bit1 = P1 enabled as output 
 *                      ...
 *                      Bit17 = P7 enabled as input 
 *                      ~Bit7 = P7 enabled as output
 *                    - uint8_t address -- device address 
 *
 * Output:         any error code resulting from i2c communication 
 *
 * Side Effects:   Inverts the polarity of any pin  
 *
 * Overview:       Writes channels to the CFG reg of the device 
 *
 * Note:           None 
 *                  
 *************************************************************************************/
uint8_t pca9543a_CFG_IO(uint8_t channels, uint8_t address){
  // Write to the CFG reg, return any error
  e.pca = writeByte(address, PCA9543A_RA_CFG, channels);
  return e.pca; 
}


 /*************************************************************************************
 * Function:        uint8_t pca9543_Polarity_Invert(uint8_t channels, uint8_t address)
 *
 * PreCondition:    I2C has been properly configured with I2C_INIT_MASTER and I2c pins
 *                  have been configured 
 *
 * Input:           Parameters: 
 *                    - uint8_t channels:
 *                      Bit0 = P0 polarity inverted  
 *                      ~Bit0 = No effect 
 *                      Bit1 = P1 polarity inverted 
 *                      ~Bit1 = No effect
 *                      ...
 *                      Bit7 = P7 polarity inverted 
 *                      ~Bit7 = nothing
 *                    - uint8_t address -- device address 
 *
 * Output:         any error code resulting from i2c communication 
 *
 * Side Effects:   Inverts the polarity of any pin who's corresponding byte is written as 1
 *
 * Overview:       Writes channels to the Polarity Inversion reg of the device 
 *
 * Note:           None 
 *                  
 *************************************************************************************/
uint8_t pca9543a_Polarity_Invert(uint8_t channels, uint8_t address){
  // Write to the Polarity Inverting reg, return any error
  e.pca=  writeByte(address, PCA9543A_RA_PI, channels);
  return e.pca; 
  }

 /*************************************************************************************
 * Function:        uint8_t pca9543_Set_Output(uint8_t channels, uint8_t address)
 *
 * PreCondition:    I2C has been properly configured with I2C_INIT_MASTER and I2c pins
 *                  have been configured 
 *
 * Input:           Parameters: 
 *                    - uint8_t channels:
 *                      Bit0 = Changes output on P0 to be high
 *                      ~Bit0 = Changes output on P0 to be low
 *                      Bit1 = Changes output on P1 to be high
 *                      ~Bit1 = Changes output on P1 to be low
 *                      ...
 *                      Bit7 = Changes output on P7 to be high
 *                      ~Bit7 = Changes output on P7 to be low
 *
 *                    - uint8_t address -- device address 
 *
 * Output:         any error code resulting from i2c communication 
 *
 * Side Effects:   Changes the state of any pin configured to be an output depending on  
 *                 channels selected
 *
 * Overview:       Selects logic level of pins configured to be outputs. Has no effect 
 *                 on pins configured to be inputs 
 *
 * Note:           None 
 *                  
 *************************************************************************************/
uint8_t pca9543a_Set_Output(uint8_t channels, uint8_t address){
  // Write to the Polarity Inverting reg, return any error
  e.pca = writeByte(address, PCA9543A_RA_OP, channels);
  return e.pca; 
  }

/*************************************************************************************
 * Function:       pca9543_Read_Input_Levels(uint8_t *levels, uint8_t address)
 *
 * PreCondition:    I2C has been properly configured with I2C_INIT_MASTER and I2c pins
 *                  have been configured 
 *
 * Input:           Parameters: 
 *                    - uint8_t * levels -- pointer to where levels should be written
 *                    - uint8_t address -- device address 
 *
 * Output:          any error code resulting from i2c communication 
 *
 * Side Effects:   Writes external logic levels to levels pointer
 *
 * Overview:       Reads External logic levels regardless of how the port is configured 
 *
 * Note:           Modifies data input 
 *                  
 *************************************************************************************/
uint8_t pca9543a_Read_Input_Levels(uint8_t *levels, uint8_t address){
  // Read from the Input Port register the external levels of the pins
  e.pca = readByte(address, PCA9543A_RA_IP, levels, 0);
  return e.pca; 

  }

/*************************************************************************************
 * Function:       pca9543_Read_Output_Levels(uint8_t *levels, uint8_t address)
 *
 * PreCondition:    I2C has been properly configured with I2C_INIT_MASTER and I2c pins
 *                  have been configured 
 *
 * Input:           Parameters: 
 *                    - uint8_t * levels -- pointer to where levels should be written
 *                    - uint8_t address -- device address 
 *
 * Output:          any error code resulting from i2c communication 
 *
 * Side Effects:   Writes current output level configuration to levels pointer
 *
 * Overview:       Reads output logic levels as they are configured in the flip flop, regardless
 *                 of reality
 *
 * Note:            
 *                  
 *************************************************************************************/
uint8_t pca9543a_Read_Output_Levels(uint8_t *levels, uint8_t address){
  // Read from the Input Port register the external levels of the pins
  e.pca= readByte(address, PCA9543A_RA_OP, levels, 0);
  return e.pca; 

  }

/*************************************************************************************
 * Function:       pca9543_Self_Test(uint8_t output_level, uint8_t address)
 *
 * PreCondition:    -I2C has been properly configured with I2C_INIT_MASTER and I2c pins
 *                  have been configured. 
 *                  - The device has not been previously configured and was just powered on
 *                  or reset. 
 *
 * Input:           Parameters: 
 *                    - uint8_t * levels -- pointer to where levels should be written
 *                    - uint8_t address -- device address 
 *
 * Output:         - Any error code resulting from i2c communication 
 *                 - Any error code resulting from configuration mismatch 
 *
 * Side Effects:  
 *
 * Overview:       1. Checks that configuration on startup matches specified configuration
 *                 2. Writes all pins to be outputs, then writes output_levels to them.
 *                 3. Reads output port register to check that configuration was set properly
 *            
 *
 * Note:            
 *                  
 *************************************************************************************/
uint8_t pca9543a_Self_Test(uint8_t output_level, uint8_t address)
  {
  uint8_t init_CFG, init_OP, init_PI, read_OP;
  uint8_t e_cfg =0x00;
  uint8_t e_op =0x00;
  uint8_t e_pi =0x00;
  uint8_t e_set_output= 0x00; 
  // Read initial configuration of RA_CFG
  e.pca =  readByte(address, PCA9543A_RA_CFG, &init_CFG, 0);
  // Read initial configuration of Output port
  e.pca=  readByte(address, PCA9543A_RA_OP, &init_OP, 0);
  // Read initial configuration of Polarity inversion register 
  e.pca =  readByte(address, PCA9543A_RA_OP, &init_PI, 0);

  // If there was no i2c error, check configuration
  if (!e.pca) {
    // if init_CFG == 0xFF do nothing, else decrement 
    if (!(init_CFG == 0xFF))
     e_cfg = 0x01; 
    if (!(init_PI == 0x00))
     e_pi = 0x02;
    if (!(init_OP == 0xFF))
     e_op = 0x04;
  }
  else return 1; 
  
  // Set all channels to ouput 
  e.pca= pca9543a_CFG_IO(0x00, address);
  // Set output level as specified 
  e.pca= pca9543_Set_Output(output_level, address);
  // Read back output level 
  e.pca =pca9543_Read_Output_Levels(&read_OP, address);
  

  if (!(read_OP == output_level))
    e_set_output = 0x08; 

  // Return and build up errors 

  return e.pca; 
  }


