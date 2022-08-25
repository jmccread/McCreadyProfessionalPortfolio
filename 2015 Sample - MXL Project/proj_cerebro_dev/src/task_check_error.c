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
 * date:	09/23/2014
 */

/******************************************************************************
*
* task_check_error()
*
******************************************************************************/

/******************************************************************************
* Includes 
******************************************************************************/
#include "task_check_error.h"
/******************************************************************************
* Globals 
******************************************************************************/
void task_check_error(void) 
{
  /******************************************************************************
  * Setup
  ******************************************************************************/
  uint8_t periph_en[6] = {B_PERIPHERAL_1_EN_I2C, B_PERIPHERAL_2_EN_I2C, 
    B_PERIPHERAL_3_EN_I2C, B_PERIPHERAL_4_EN_I2C, A_PERIPHERAL_5_EN_I2C,
    A_MOTOR_CTRL_EN_I2C};
  uint8_t k =0; 
  LED_SEL &= ~PIN_LED;
  LED_OUT &= ~PIN_LED;
  LED_DIR |=  PIN_LED;
  while(1) 
    {
    // Wait for a signal that the bus has broken
    OS_WaitBinSem(CHECK_ERROR_P, OSNO_TIMEOUT);
    LED_OUT |=  PIN_LED_2;
    if(e.f_open | e.f_close | e.f_write)
     {
      clear_error_sd();
      // Figure out what to do here 
     }
    
    if (!check_e()) 
    {
     clear_error_i2c(); 

     // In the event of a failure Reset the i2c bus and reconfigure isolation state
     I2C_Init_Master(USCI_B_I2C_CLOCKSOURCE_SMCLK,
                   SMCLK_SPD,
                   USCI_B_I2C_SET_DATA_RATE_400KBPS);
     init_isolation_state();
     delay_ms(100);

    // Setup Gyro and ACC
    e.iso= IO_E_ENABLE(A_GYRO_XCL_PT_EN, IO_E_A);

    e.acc = configure_ADXL345(ACC);
    e.gyro = configureL3G4200D(GYRO);

    e.iso = IO_E_DISABLE(A_GYRO_XCL_PT_EN, IO_E_A); 
  
   /*
    // Setup 5x magnetometers 
    for(k=0; k<5; k++) {
      if (k<4) { // Should be k<4
        // Un-Isolate
        if (check_e()) e.iso = IO_E_ENABLE(periph_en[k], IO_E_B);
        if (check_e()) e.mag = initialize_HMC5883(); 
        // Isolate
        if (check_e())  e.i2c = IO_E_DISABLE(periph_en[k], IO_E_B); 
      }
 
    else if (k == 4) {
      // Un-Isolate
      if (check_e()) e.iso = IO_E_ENABLE(periph_en[k], IO_E_A);
      if (check_e())  e.mag = initialize_HMC5883();
      // Isolate
      if (check_e()) e.iso = IO_E_DISABLE(periph_en[k], IO_E_A); 
      } 
      delay_ms(30);
      } */
    } 
     if (e.gps_r)
     {
      // Configure GPS
      UART_Channel_Select(UART_DEVICE_CONNECT); 
      IO_E_ENABLE(A_GPS_UART_EN, IO_E_A);
      start_device_uart(9600);
      initialize_lea_6(); 
      IO_E_DISABLE(A_GPS_UART_EN, IO_E_A);  

      clear_error_gps();
     }

     // Return to the scheduler
     OS_Yield();
     }
}


void clear_error_i2c()
{
  e.broke =0; 
  e.i2c =0;
  e.gyro = 0; 
  e.acc = 0; 
  e.adc = 0; 
  e.mag = 0; 
  e.iso = 0; 
  e.pca = 0; 
  e.tca = 0; 
  e.bmp = 0; 
  e.i2c_e_msg = 0; 
  }

 void clear_error_sd() 
 {
  e.f_write = 0; 
  e.f_close = 0; 
  e.f_open = 0;  
 }

 void clear_error_gps()
 {
  e.gps_r = 0; 
 }



