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
 * date:	07/30/2014
 */

/******************************************************************************
*
* task_measure_high_f()
*
******************************************************************************/

/******************************************************************************
* Includes 
******************************************************************************/
#include "task_measure_high_f.h"

/******************************************************************************
* Globals 
******************************************************************************/
// Scale factors for each magnetometer on each axis determined by magnetometer 
//calibration procedure 
float mag_scale[15]; 

// Configurations to unisolate peripheral adc
uint8_t periph_en[6] = {B_PERIPHERAL_1_EN_I2C, B_PERIPHERAL_2_EN_I2C, 
  B_PERIPHERAL_3_EN_I2C, B_PERIPHERAL_4_EN_I2C, A_PERIPHERAL_5_EN_I2C,
  A_MOTOR_CTRL_EN_I2C};
uint8_t k;
void task_measure_high_f() {

  /*float median = 0;
  int greater_cnt = 0;
  int less_cnt = 0;
  int i, j;
  

  /******************************************************************************
  * Setup
  ******************************************************************************/
  I2C_SEL |= (PIN_SDA+PIN_SCL);   
 
   // Setup I2C according to config.h
   I2C_Init_Master(USCI_B_I2C_CLOCKSOURCE_SMCLK,
                   16000000,
                   USCI_B_I2C_SET_DATA_RATE_100KBPS);


  
  // Setup Gyro and ACC
  if (check_e()) e.iso= IO_E_ENABLE(A_GYRO_XCL_PT_EN, IO_E_A);

  if (check_e()) e.acc = configure_ADXL345(ACC);
  if (check_e()) e.gyro = configureL3G4200D(GYRO);

  if (check_e()) e.iso = IO_E_DISABLE(A_GYRO_XCL_PT_EN, IO_E_A); 
  
  /*
  // Setup 5x magnetometers 
  for(k=0; k<5; k++) {
    if (k<4) { // Should be k<4
      // Un-Isolate
      if (check_e()) e.iso = IO_E_ENABLE(periph_en[k], IO_E_B);
      if (check_e()) e.mag = Calibrate_HMC5883(&mag_scale[k*3], &mag_scale[k*3+1], &mag_scale[k*3+2]);
      if (check_e()) e.mag = initialize_HMC5883();
      
      // Isolate
      if (check_e())  e.i2c = IO_E_DISABLE(periph_en[k], IO_E_B); 
      }
   
   else if (k == 4) {
      // Un-Isolate
      if (check_e()) e.iso = IO_E_ENABLE(periph_en[k], IO_E_A);

      if (check_e()) e.mag = Calibrate_HMC5883(&mag_scale[k*3], &mag_scale[k*3+1], &mag_scale[k*3+2]);
      if (check_e())  e.mag = initialize_HMC5883();

      // Isolate
      if (check_e()) e.iso = IO_E_DISABLE(periph_en[k], IO_E_A); 
      } 
   }
   */
   
  while(1) 
    {
    /******************************************************************************
    * Update time stamp for High frequency measurements
    ******************************************************************************/ 
    data.HF_t_elap = update_time( &(data.h_HF) , &(data.m_HF), &(data.s_HF), &(data.new_gps_HF)); 
   
    /******************************************************************************
    * Sample Storm Peripheral Board Sensors (Gyro and Acc) 
    ******************************************************************************/  
    // Unisolate 
    if (check_e()) e.iso = IO_E_ENABLE(A_GYRO_XCL_PT_EN, IO_E_A);
    else OS_Yield(); 
 
 
    // Measure Gyro and Accelerometer 
    if (check_e()) e.gyro = measureL3G4200D(GYRO, &data.gyro_temp,
      &data.gyro_dps[0], &data.gyro_dps[1], &data.gyro_dps[2]); 
    else OS_Yield();

    if (check_e()) e.acc = measure_ADXL345(&data.acc_g[0] , &data.acc_g[1], &data.acc_g[2], ACC);
    else OS_Yield(); 
    /*
    if (check_e()) e.iso = IO_E_DISABLE(A_GYRO_XCL_PT_EN, IO_E_A); 
    else OS_Yield(); */

    /******************************************************************************
    * Sample 5 Periph Sensor Boards and MTR_CTR (6X adc) 
    ******************************************************************************/  
    /*if (check_e()) sample_peripherals(); // (It looks less menacing)
    else OS_Yield(); */

    /******************************************************************************
    * Signal that new measurements are available and give up control of scheduler
    ******************************************************************************/ 
    OSSignalBinSem(NEW_HIGH_F_P); 
    
    // Return to the scheduler 
    OSSignalBinSem(LOG_HIGH_F_P); // TO DO: Move back to Stamp comm task or delete entirely 
    OS_Yield(); 
    }
  } /* task_measure_high_f */ 


/*************************************************************************************
 * Function:        sample_peripherals() 
 *
 * PreCondition:    1. Setup i2c.
 *                  2. Configuration of isolation state 
 *                  3. Configuration of calibration magentometers
 *                  
 * Input:           None
 *  
 * Output:          Possibly outputs error messages (not implimented) 
 *
 * Side Effects:    Writes collected data to global telem struct
 *
 * Overview:        Cycles through 6 peripheral boards un isolating and isolating each
 *                  each one and collecting data in between 
 *
 * Note:            
 *
*************************************************************************************/
uint8_t sample_peripherals() 
  {
  /******************************************************************************
    * Variables 
  ******************************************************************************/ 
  uint8_t i, j;                               // Index
  uint16_t m[24];                              // Raw adc measurements
  int16_t raw_mag[15];                         
  float adc_xfer =(float)(AD799x_Vref/4096);  // Convert ADC output to voltages
  for(i = 0; i < 5; i++) { 
    //******************************************************************************
    // Sample 
    //*****************************************************************************/ 
   
    //******************************************************************************
    //*
    //* IO_E_B
    //*
    //******************************************************************************/ 
    if (i<4) 
    //if(i==0)
    {
      // Un-Isolate
      if (check_e()) e.iso= IO_E_ENABLE(periph_en[i], IO_E_B);
      else return -1;

      // Sample ADCs
      if (check_e()) e.adc = command_mode_read(CH1|CH2|CH3|CH4, PERIPH_ADC , &m[i*4]); 
      else return -1; 
      
      // Sample Magnetometers
      if (check_e())  e.mag= getHeading(&raw_mag[i*3], &raw_mag[i*3+1], &raw_mag[i*3+2]);
      else return -1; 
      
      // Isolate
      if (check_e()) e.iso = IO_E_DISABLE(periph_en[i], IO_E_B);
      else return -1;
      data.adc_temp[i] = LM20_xfer((float)(m[(i*4)+3]*adc_xfer)); 
    } 

    /******************************************************************************
    *
    * IO_E_A
    *
    ******************************************************************************/ 
    else if (i==4) { 
      if (check_e()) e.iso = IO_E_ENABLE(periph_en[i], IO_E_A);
      else return -1;
      /******************************************************************************
      * Sample ADC and magnetometer 
      ******************************************************************************/
     
     // Sample ADC
     if (check_e()) e.adc = command_mode_read(CH1|CH2|CH3|CH4, PERIPH_ADC ,&m[i*4]);
     else return -1; 
     
      // Sample Magnetometers
      if (check_e()) e.mag = getHeading(&raw_mag[i*3], &raw_mag[i*3+1], &raw_mag[i*3+2]);
      else return -1; 

      if (check_e()) e.iso = IO_E_DISABLE(periph_en[i], IO_E_A);
      else return -1; 
    } // else if (i == 4)

   //else if (i==5) {
       /******************************************************************************
       * Sample ADC=
       *****************************************************************************/
       //if (check_e()) e.adc = command_mode_read(CH1|CH2|CH3|CH4, PERIPH_ADC ,&m[i*4]);
       //else return -1;
       //if (check_e())  e.iso =IO_E_DISABLE(periph_en[i], IO_E_A);
       //else return -1;
         //}  

    if (i<5) {
      // Set temp 
      data.adc_temp[i] = LM20_xfer((float)(m[(i*4)+3]*adc_xfer)); 
      for (j = 0; j<3; j++) 
      {
        data.photo_V[(i*3)+j] = (float)(m[(i*4)+j]*adc_xfer);
        data.mag_g[(i*3)+j] = (float)(m[(i*4)+j]*adc_xfer);
       
        // Convert Raw measures into calibrated ones - ENABLE BEFORE FLIGHT
        if (i<4) data.mag_g[i*3+j] = (float)raw_mag[i*3+j]*mag_scale[i*3+j]/hmc5883_gain;
      } // for
    }// if

//   else if (i == 5) {
//       for (j = 0; j<3; j++) {
//          data.mtr_telem[j] = (float)(m[(i*3)+j]*adc_xfer);
//       }
//    }

    } //for
  return 1; 
  } /* sample_peripherals()*/ 

  
float LM20_xfer(float temp_V) {
  return ((float)-1481.96+sqrt(2.1962*1000000+(1.8639-temp_V)/(0.00000388)));
}
