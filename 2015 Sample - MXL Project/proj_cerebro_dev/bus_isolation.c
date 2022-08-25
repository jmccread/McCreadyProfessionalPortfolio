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
#include "bus_isolation.h"

/******************************************************************************
*
* Declarations  
*
******************************************************************************/

 /*************************************************************************************
 * Function:        uint8_t init_isolation_state()
 *
 * PreCondition:    I2C was properly configured with correct pins selected 
 *
 * Input:           None
 *
 * Output:          Any I2C error which occred 
 *
 * Side Effects:    Sets isolation states as follows for shared i2c bus, uart bus, and 
 *                  connected devices. 
 *
 * Overview:       Sets isolation states as follows:
 *                    - Select CMD_I2C with I2C_MUX (TCA9548a)
 *                    - Configure IO_E_A (pca9543a 0x??) channels all to be low outputs
 *                      (all devices on IO_E_A are off)
 *                    - Configure IO_E_B (pca9543a 0x??) channels all to be low outputs
  *                     (all devices on IO_E_A are off)
 *                    - Setup UART_MUX (ltc1393) to talk to GPS and XBee
 *                  
 *************************************************************************************/
uint8_t init_isolation_state() {
  // Variable Definitons 
  uint8_t err = 0x00; 
  // Select CMD_I2C with I2C_MUX (TCA9548a)
  err= tca9548_channel_select(CMD_I2C_EN, I2C_MUX);

  // Stop if an error Occurred 
  //if (!err){
    //Configure IO_E_A (pca9543a) channels all to be low outputs
    //err = pca9543a_Self_Test(uint8_t output_level, uint8_t address);
    err = pca9543a_CFG_IO(0x00, IO_E_A);
    err = pca9543a_Set_Output(0x00, IO_E_A);

    // Configure IO_E_B (pca9543) channels all to be low outputs
    //err = pca9543a_Self_Test(uint8_t output_level, uint8_t address);
    err = pca9543a_CFG_IO(0x00, IO_E_B);
    err = pca9543a_Set_Output(0x00, IO_E_B);
    
    // Delay everything turning on for 10 ms
    delay_ms(100);

    // Setup UART_MUX (ltc1393) to talk to GPS and XBee
    err= UART_Channel_Select(UART_DEVICE_CONNECT); 
    //err = ltc1393_channel_select(UART_DEVICE_CONNECT, UART_MUX);
    
    // Power up devices on the IO Expanders 
    err =power_IO_E_AB();
  //}

  return err; 
}

 /*************************************************************************************
 * Function:        uint8_t power_IO_E_AB(void)
 *
 * PreCondition:    I2C was properly configured with correct pins selected 
 *                  init_isolation_state() ran without error
 *
 * Input:           None
 *
 * Output:          Any I2C error which occured  
 *
 * Side Effects:   Lots of LEDs turn on 
 *
 * Overview:       Powers Up devices on IO_E A and B
 *                 Delay after doing this for at least 0.03 seconds
 *************************************************************************************/
uint8_t power_IO_E_AB(){
  uint8_t err =0x00;  
  // Disable all previous configurations before multitasking 
  //err = IO_E_DISABLE(0xFF, IO_E_A);
  //err = IO_E_DISABLE(0xFF, IO_E_B);
  // Turn pwr on to all devices 
  err = IO_E_ENABLE(A_FULL_PWR_EN, IO_E_A);
  err = IO_E_ENABLE(B_FULL_PWR_EN, IO_E_B);

  return err; 
  }


/*************************************************************************************
 * Function:        uint8_t IO_E_ENABLE(uint8_t config, uint8_t address)
 *
 * PreCondition:    I2C was properly configured with correct pins selected 
 *                  init_isolation_state() ran without error
 *                  uint8_t power_IO_E_AB() ran without error
 *
 * Input:           None
 *
 * Output:          Any I2C error which occurred  
 *
 * Side Effects:   
 *
 * Overview:       Enables a device on the either IO_E A or B
 *                 Does not overwrite existing configuration 
 *                  
 *************************************************************************************/
uint8_t IO_E_ENABLE(uint8_t config, uint8_t address){
  uint8_t err = 0x00; 
  uint8_t old_cfg =0x00; 
  // Read output levels as to not power off any devices
  err = pca9543a_Read_Output_Levels(&old_cfg, address);
  
  //if (!e) {
    // Logic to Preserve any channels that were already enabled (keep devices powered on)
    config = config | old_cfg;
    // Set new configuration 
    err = pca9543a_Set_Output(config, address);
    err = pca9543a_Read_Output_Levels(&old_cfg, address);
    //}
  return err; 
  }

/*************************************************************************************
 * Function:        uint8_t IO_E_DISABLE(uint8_t config, uint8_t address)
 *
 * PreCondition:    I2C was properly configured with correct pins selected 
 *                  init_isolation_state() ran without error
 *                  uint8_t power_IO_E_AB() ran without error
 *
 * Input:           Parameters:
 *                  
 *
 * Output:          Any I2C error which occurred  
 *
 * Side Effects:   Lots of LEDs turn on 
 *
 * Overview:       Powers Up devices on IO_E A and B
 *                  
 *************************************************************************************/
uint8_t IO_E_DISABLE(uint8_t config, uint8_t address){
  // Variables 
  uint8_t err = 0x00; 
  uint8_t old_cfg, blah;

  // Get current configuration as to not disable other devices 
  err = pca9543a_Read_Output_Levels(&old_cfg, address);

   // Logic to Preserve any channels that were already enabled (keep devices powered on)
   // besides those being disabled 
   config = old_cfg & ~config; 

   // Set new configuration 
   err = pca9543a_Set_Output(config, address);
   
   return err; 
  }

/*************************************************************************************
 * Function:        UART_Channel_Select(uint8_t channel)
 *
 * PreCondition:    -I2C was properly configured with correct pins selected 
 *                  -init_isolation_state() ran without error
 *                  -uint8_t power_IO_E_AB() ran without error
 *                  -When UART_DEVICE_CONNECT is configured only one UART device is un
 *                   isolated on the bus.
 *                  -When STAMP_MSP_CONNECT is configured all UART devices are isolated and
 *                   only the MSP430 and STAMP may communicate on the bus 
 *
 * Input:           Parameters:
 *                    -uint8_t channel: Any configuration in ltc1339.h or in bus isolation.j
 *                  
 *
 * Output:          Any I2C error which occurred  
 *
 * Side Effects:    None
 *
 * Overview:        Changes UART modes between MSP430 to STAMP communication and MSP430
 *                  or STAMP to an unisolated device on the UARTB Bus. 
 *                  
 *************************************************************************************/
 uint8_t UART_Channel_Select(uint8_t channel){
  uint8_t err =0x00; 
  // Select only one UART mode at a time, turn all others off
  err =ltc1393_channel_select(LTC1393_ALL_OFF, UART_MUX); 
  // Select new UART Mode 
  err =ltc1393_channel_select(channel, UART_MUX);
  return err; 
  }

/*************************************************************************************
 * Function:        I2C_Channel_Select(uint8_t channel)
 *
 * PreCondition:    -I2C was properly configured with correct pins selected 
 *                  
 * Input:           Parameters:
 *                    -uint8_t channel: Any configuration in bus isolation.h
 *                  
 *
 * Output:          Any I2C error which occurred  
 *
 * Side Effects:    None
 *
 * Overview:        Changes configuration between the two I2C buses either CMD or TELEM
 *                
 *                  
 *************************************************************************************/
 uint8_t I2C_Channel_Select(uint8_t channel){
  uint8_t err =0x00; 
  // Select only one UART mode at a time, turn all others off
  err =tca9548_channel_select(0x00, I2C_MUX); 
  // Select new UART Mode 
  err =tca9548_channel_select(channel, I2C_MUX);
  return err;
  }

 void test_isolation() {
 // Dinking around 
 I2C_Channel_Select(TELEM_I2C_EN);
 I2C_Channel_Select(I2C_DISABLE);
 I2C_Channel_Select(CMD_I2C_EN);
 IO_E_ENABLE(A_GYRO_XCL_PT_EN, IO_E_A);
 IO_E_DISABLE(A_GYRO_XCL_PT_EN, IO_E_A);
 I2C_Channel_Select(I2C_DISABLE);
 }