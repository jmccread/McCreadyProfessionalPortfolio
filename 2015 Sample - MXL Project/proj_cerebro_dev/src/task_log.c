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
 * date:	10/31/2014
 */
#include "task_log.h"

/*************************************************************************************
 * Function:        void task_log_high_f(void)
 *
 * PreCondition:    Correct # define exists to decide which bus SPI is one
 *                  task_stamp_comm() has run 
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    Writes high frequency data obtained from task_measure_high_f 
 *                  15 photo diode measurements, 15 magentometer measurements, 
 *                  angular rate, acceleration   
 *
 * Overview:        
 *
 * Note:           TO DO ADD LINE THAT PRINTS README.txt to explain what info is 
 *                  BEING WRITTEN 
*************************************************************************************/
void task_log_high_f(void)
{
  F_FILE * photo_file; 
  F_FILE * photoreadme; 
  F_FILE * dynamicsreadme; 
  F_FILE * dynamics_file; 
  F_FILE * mag_file; 
  uint16_t wrote_B =0; 
  // Initialize 
  csk_sd_open();
  f_initvolume();
  csk_led_status_open(PIN_LED_1);  
  
  // Write photo_read.me
  // toggle chip select 
  CS_SEL &= ~CS_SD;
  CS_DIR |=  CS_SD;
  CS_OUT &=  ~CS_SD;
  wrote_B =0; 

  /*
  //  Open file
  photoreadme = f_open("PHOTORME.txt", "w");

  if (check_sd_e())
      {
      if (photoreadme)
      {
        csk_led_status_on(PIN_LED);
        sprintf(data.data_str, "Real time H:M:S, SYS TIME (s), P0_A (V), P0_B (V), P0_C (V), P1_A (V), P1_B (V), P1_C (V), P2_A (V), P2_B (V), P2_C (V), P3_A (V), P3_B (V), P_3C (V), P4_A (V), P4_B (V), P4_C (V)"); 
        wrote_B = f_write(&data.data_str, 1, strlen(data.data_str), photoreadme ); 
        if (wrote_B == 0) {e.f_write =1;  }  
      
        e.f_close = f_close(photoreadme);
        if (e.f_close == F_NO_ERROR) e.f_close = 0;  
        else 
          e.f_close = 1; 
        csk_led_status_off(PIN_LED);
      }
      else 
        e.f_open = 1; 
    }
    else OS_Yield();

    */

    
  //  Open file
  dynamicsreadme = f_open("DYNAMRME.txt", "w");

  if (check_sd_e())
      {
      if (dynamicsreadme)
      {
        csk_led_status_on(PIN_LED);
        sprintf(data.data_str, "Real time H:M:S, SYS TIME (s), GyroX (dps), GyroY (dps), GyroZ (dps), AccX (g), AccY (g), AccZ (g)"); 
        wrote_B = f_write(&data.data_str, 1, strlen(data.data_str), dynamicsreadme ); 
        if (wrote_B == 0) {e.f_write =1;  }  
      
        e.f_close = f_close(dynamicsreadme);
        if (e.f_close == F_NO_ERROR) e.f_close = 0;  
        else 
          e.f_close = 1; 
        csk_led_status_off(PIN_LED);
      }
      else 
        e.f_open = 1; 
    }
    else OS_Yield();

while(1)
  {
    // Wait for the SD card to become available
    OS_WaitBinSem(LOG_HIGH_F_P, OSNO_TIMEOUT);

    // toggle chip select 
    CS_SEL &= ~CS_SD;
    CS_DIR |=  CS_SD;
    CS_OUT &=  ~CS_SD;
    wrote_B =0; 
    /*
    //  Open file
    photo_file = f_open("Photo.txt", "a");
    if (check_sd_e())
      {
      if (photo_file)
      {
        csk_led_status_on(PIN_LED);
        sprintf(data.data_str, "%u:%u:%f, %f, %f, %f, %f,  %f,  %f,  %f,  %f,  %f,  %f,  %f,  %f,  %f,  %f,  %f,  %f\r\n", 
        data.h_HF, data.m_HF, data.s_HF, data.HF_t_elap, 
        data.photo_V[0], data.photo_V[1],  data.photo_V[2],  data.photo_V[3],  data.photo_V[4],  data.photo_V[5],  
        data.photo_V[6],  data.photo_V[7],  data.photo_V[8],  data.photo_V[9],  data.photo_V[10],  data.photo_V[11],  
        data.photo_V[12], data.photo_V[13],  data.photo_V[14]); 

        wrote_B = f_write(&data.data_str, 1, strlen(data.data_str), photo_file); 
        if (wrote_B == 0) {e.f_write =1;  }  
      
        e.f_close = f_close(photo_file);
        if (e.f_close == F_NO_ERROR) e.f_close = 0;  
        else 
          e.f_close = 1; 
        csk_led_status_off(PIN_LED);
      }
      else 
        e.f_open = 1; 
    }
    else OS_Yield();
    */
    if (check_sd_e())
      {
      //  Open file
      dynamics_file = f_open("Dynamics.txt", "a");
      if (check_sd_e())
      {
        if (dynamics_file)
        {
          csk_led_status_on(PIN_LED);
          sprintf(data.data_str, "%u:%u:%f, %f, %f, %f, %f,  %f,  %f,  %f\r\n", 
          data.h_HF, data.m_HF, data.s_HF, data.HF_t_elap, 
          data.gyro_dps[0], data.gyro_dps[1], data.gyro_dps[2],
          data.acc_g[0], data.acc_g[1], data.acc_g[2]); 


          wrote_B = f_write(&data.data_str, 1, strlen(data.data_str), dynamics_file); 
          if (wrote_B == 0) {e.f_write =1; }  

          e.f_close = f_close(dynamics_file);
          if (e.f_close == F_NO_ERROR) e.f_close = 0;  
          else
           e.f_close = 1;  
           csk_led_status_off(PIN_LED);
        }
        else 
          e.f_open = 1;
      }
      }
    else OS_Yield(); 
  /*
  if (check_sd_e())
      {
      
      //  Open file
      mag_file = f_open("Magnetometer.txt", "a");

      if (check_sd_e())
      {
        if (mag_file)
        {
          sprintf(data.data_str, "%u:%u:%f, %f, %f, %f, %f,  %f,  %f,  %f,  %f,  %f,  %f,  %f,  %f,  %f,  %f,  %f,  %f\r\n", 
          data.h_HF, data.m_HF, data.s_HF, data.HF_t_elap, 
          data.mag_g[0], data.mag_g[1],  data.mag_g[2],  data.mag_g[3],  data.mag_g[4],  data.mag_g[5],  
          data.mag_g[6],  data.mag_g[7],  data.mag_g[8],  data.mag_g[9],  data.mag_g[10],  data.mag_g[11],  
          data.mag_g[12], data.mag_g[13],  data.mag_g[14]); 
      
          wrote_B = f_write(&data.data_str, 1, strlen(data.data_str), mag_file); 
          if (wrote_B == 0) {e.f_write =1; }  

          e.f_close = f_close(mag_file);
          if (e.f_close == F_NO_ERROR) e.f_close = 0;  
          else {e.f_close == 1; };  
        } ss
        else e.f_open = 1; 
      }
    else OS_Yield();    
    
    } */
    // Yield to OS
    OS_Yield(); 
  }
}

/*************************************************************************************
 * Function:        void task_log_low_f(void)
 *
 * PreCondition:    Correct # define exists to decide which bus SPI is one
 *                  task_stamp_comm() has run 
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    Writes low 
 * Overview:        
 *
 * Note:           TO DO ADD LINE THAT PRINTS README.txt to explain what info is 
 *                  BEING WRITTEN 
*************************************************************************************/
void task_log_low_f()
{
  F_FILE * bmp_file; 
  F_FILE * bmprme; 
  F_FILE * gps_file; 
  F_FILE * gps_parsed;
  F_FILE * gpsrme; 
  F_FILE * misc_telem; 
  uint16_t wrote_B =0; 
  int lentest=0;

  // Initialize 
  csk_sd_open();
  f_initvolume();

      // toggle chip select 
    CS_SEL &= ~CS_SD;
    CS_DIR |=  CS_SD;
    CS_OUT &=  ~CS_SD;
    wrote_B =0; 

    //  Open file
    bmprme = f_open("BMPRME.txt", "w");

    if (check_sd_e())
        {
        if (bmprme)
        {
          csk_led_status_on(PIN_LED);
          sprintf(data.data_str, "Real time H:M:S, SYS TIME (s), pressure (Pa), Interior temp (0.1 deg C)"); 
          wrote_B = f_write(&data.data_str, 1, strlen(data.data_str), bmprme ); 
          if (wrote_B == 0) {e.f_write =1;  }  
      
          e.f_close = f_close(bmprme);
          if (e.f_close == F_NO_ERROR) e.f_close = 0;  
          else 
            e.f_close = 1; 
          csk_led_status_off(PIN_LED);
        }
        else 
          e.f_open = 1; 
      }
      else OS_Yield();


      gpsrme = f_open("GPSLOGRME.txt", "w");
      if (check_sd_e())
        {
        if (gpsrme)
        {
          csk_led_status_on(PIN_LED);
          sprintf(data.data_str, "hours, min, sec, lat, long, alt (m)"); 
          wrote_B = f_write(&data.data_str, 1, strlen(data.data_str), gpsrme ); 
          if (wrote_B == 0) {e.f_write =1;  }  
      
          e.f_close = f_close(bmprme);
          if (e.f_close == F_NO_ERROR) e.f_close = 0;  
          else 
            e.f_close = 1; 
          csk_led_status_off(PIN_LED);
        }
        else 
          e.f_open = 1; 
      }
      else OS_Yield();

  while(1)
  {
    // Wait for new data
    OS_WaitBinSem(LOG_LOW_F_P, OSNO_TIMEOUT);

    // toggle chip select 
    CS_SEL &= ~CS_SD;
    CS_DIR |=  CS_SD;
    CS_OUT &=  ~CS_SD;
    wrote_B =0; 

    if(check_sd_e())
    {
      //  Open file
      bmp_file = f_open("BMP180.txt", "a");
    
      if (bmp_file)
      {
        /*
        sprintf(data.data_str, "%u:%u:%f, %f, %li, %hi, %f, %f, %f, %f, %f, %u\r\n", 
        data.h_LF, data.m_LF, data.s_LF, data.LF_t_elap, data.bmp_pressure, data.bmp_temp,
        data.adc_temp[0], data.adc_temp[1], data.adc_temp[2], data.adc_temp[3], data.adc_temp[4], 
        data.heater_on); */
        
        // Without photodiode crown
        sprintf(data.data_str, "%u:%u:%f, %f, %li, %hi, %u\r\n", 
        data.h_LF, data.m_LF, data.s_LF, data.LF_t_elap, data.bmp_pressure, data.bmp_temp, 
        data.heater_on); 

        wrote_B = f_write(&data.data_str, 1, strlen(data.data_str), bmp_file); 
        if (wrote_B == 0) {e.f_write =1; }  
      
        e.f_close = f_close(bmp_file);
        if (e.f_close == F_NO_ERROR) e.f_close = 0;  
        else 
          e.f_close = 1; 
      }
      else 
        e.f_open = 1; 
    }
    else OS_Yield(); 
     
    if (check_sd_e())
    {
      //  Open file
      gps_file = f_open("GPSLOG.txt", "a");
      if (check_sd_e()){
        if (gps_file)
        {
          if (!(data.nmea_str[strlen(data.nmea_str)-1] == '\n'))
         {  
           data.nmea_str[strlen(data.nmea_str)] = '\n'; 
         }
          wrote_B = f_write(&data.nmea_str, 1, strlen(data.nmea_str), gps_file); 
   
          if (wrote_B == 0) {e.f_write =1;  }  

          e.f_close = f_close(gps_file);
          if (e.f_close == F_NO_ERROR) e.f_close = 0;  
         else 
           e.f_close = 1;  
        }
        else 
          e.f_open = 1;
      }
    }
    else OS_Yield(); 

    gps_parsed = f_open("Parsed_GPS.txt", "a");
    if (check_sd_e())
    {
      if (gps_parsed)
      {
        /*
        sprintf(data.data_str, "%u:%u:%f, %f, %li, %hi, %f, %f, %f, %f, %f, %u\r\n", 
        data.h_LF, data.m_LF, data.s_LF, data.LF_t_elap, data.bmp_pressure, data.bmp_temp,
        data.adc_temp[0], data.adc_temp[1], data.adc_temp[2], data.adc_temp[3], data.adc_temp[4], 
        data.heater_on); */
        sprintf(data.data_str, "%u:%u:%f, %f, %u, %u,  %f, %f, %f, %f\r\n", 
        data.h_LF, data.m_LF, data.s_LF, data.LF_t_elap, data.hours, data.mins, data.secs, data.lon, data.lat, data.alt); 

        wrote_B = f_write(&data.data_str, 1, strlen(data.data_str), gps_parsed); 
        if (wrote_B == 0) {e.f_write =1; }  
      
        e.f_close = f_close(gps_parsed);
        if (e.f_close == F_NO_ERROR) e.f_close = 0;  
        else 
          e.f_close = 1; 
      }
      else 
        e.f_open = 1; 
    }
    else OS_Yield(); 
    
    /*
    if (check_sd_e())
      {
      //  Open file
      misc_telem = f_open("Misc_telem.txt", "a+");
      if (check_sd_e()){
        if (misc_telem)
        {
          sprintf(data.data_str, ); 

          wrote_B = f_write(&data.data_str, 1, strlen(data.data_str), gps_file); 
          if (wrote_B == 0) {e.f_write =1;  OS_Yield(); }  

          e.f_close = f_close(misc_telem);
          if (e.f_close == F_NO_ERROR) e.f_close = 0;  
          else {e.f_close = 1; };  
        }
        else e.f_open = 1;
      }
     }
     else OS_Yield(); 
    */

    // Yield to OS
    OS_Yield(); 
   }
}