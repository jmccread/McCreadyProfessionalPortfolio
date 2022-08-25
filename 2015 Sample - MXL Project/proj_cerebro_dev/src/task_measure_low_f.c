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
* task_measure_low_f()
*
******************************************************************************/

/******************************************************************************
* Includes 
******************************************************************************/
#include "task_measure_low_f.h"
#include "uart.h"
/******************************************************************************
* Globals 
******************************************************************************/
gps_struct gps_msg; 

void task_measure_low_f(void) {
   float send_gps[3]; 
   uint32_t pres; 
  /******************************************************************************
  * Setup
  ******************************************************************************/
  I2C_SEL |= (PIN_SDA+PIN_SCL);   
 
   // Setup I2C according to config.h
   #ifdef I2C_UCB0
   I2C_Init_Master(USCI_B_I2C_CLOCKSOURCE_SMCLK,
                   SMCLK_SPD,
                   USCI_B_I2C_SET_DATA_RATE_400KBPS);
   #endif

  // Setup BMP180 
  if (check_e()) e.iso = IO_E_ENABLE(A_GYRO_XCL_PT_EN, IO_E_A);
  bmp085Calibration();
  if (check_e()) e.iso = IO_E_DISABLE(A_GYRO_XCL_PT_EN, IO_E_A); 

  // Configure GPS
  UART_Channel_Select(UART_DEVICE_CONNECT); 
  IO_E_ENABLE(A_GPS_UART_EN, IO_E_A);
  start_device_uart(9600);
  initialize_lea_6(); 
  IO_E_DISABLE(A_GPS_UART_EN, IO_E_A); 
  while(1) 
    {
     /******************************************************************************
    * Wait for trigger defined in init.c timera0 ISR
    ******************************************************************************/  
    OS_WaitBinSem(COLLECT_LOW_F_P, OSNO_TIMEOUT);

    if (check_e()) { 
      /******************************************************************************
      * Sample Storm Peripheral Board BMP180 
      ******************************************************************************/  
      if (check_e()) IO_E_ENABLE(A_GYRO_XCL_PT_EN, IO_E_A);
      else OS_Yield();
      // Temperature measure is in 0.1 degrees C
      bmp085ReadUT();
      // Wait at least 4.5ms, assuming OS_timer at 1 ms ticks
      OS_Delay(5);
     // Adding Temperature into global data container, change for specific applications 
      data.bmp_temp = bmp085GetTemperature();
  
      // Pressure measured in Pa
      bmp085ReadUP();
      // Wait for conversion, delay time dependent on OSS, assuming  1 ms ticks
      // bitshifted 0 corresponds to over sampling setting 
      OS_Delay(5);

      // Adding pressure into global data container, change for specific applications 
      data.bmp_pressure = bmp085GetPressure();
      //pres = (uint32_t) data.bmp_pressure;
      // if (check_e()) IO_E_DISABLE(A_GYRO_XCL_PT_EN, IO_E_A); 
      //else OS_Yield();

     /******************************************************************************
     * Transmit new data 
     ******************************************************************************/  
     //UART_Channel_Select(STAMP_MSP_CONNECT);
     //start_device_uart(460800);

     //uart_transmit_int16(&UART_A1,  &data.bmp_temp, TEMP_IDENT,  1);
     //uart_transmit_uint32(&UART_A1,  &pres, PRESSURE_IDENT,  1);
     }
    else OS_Yield();
    
    /******************************************************************************
    * Sample GPS for position and time data 
    ******************************************************************************/  
    UART_Channel_Select(UART_DEVICE_CONNECT); 
    if (check_e()) IO_E_ENABLE(A_GPS_UART_EN, IO_E_A);
    else OS_Yield();
    start_device_uart(9600);
    preprocgps(&gps_msg);
    e.gps_r =poll_gpgga();

    if (e.gps_r) 
      {
       updateGPS(&gps_msg, &data.nmea_str);
       postprocgps(&gps_msg); 
       
       //send_gps[0]= gps_msg.lon_f;
       //send_gps[1] = gps_msg.lat_f; 
       //send_gps[2] = gps_msg.alt_f_m; 

       data.lon = gps_msg.lon_f;
       data.lat = gps_msg.lat_f; 
       data.alt = gps_msg.alt_f_m; 

       data.hours = gps_msg.hour; 
       data.mins = gps_msg.min; 
       data.secs = gps_msg.sec; 

       if (gps_msg.fix_status == '\0')
       {
        data.new_gps_LF = 0; 
        data.new_gps_HF = 0; 
       }
       else 
       {
        data.new_gps_LF = 1; 
        data.new_gps_HF = 1;
       }
       e.gps_r = 0; 


       /******************************************************************************
       * Transmit new data 
       ******************************************************************************/  
       //UART_Channel_Select(STAMP_MSP_CONNECT);
       //start_device_uart(460800);
       //uart_transmit_float(&UART_A1,  &send_gps, GPS_IDENT,  3);
       }
    else 
      {
      e.gps_r = 1; 
      OSSignalBinSem(CHECK_ERROR_P);
      } 
    if (check_e()) IO_E_DISABLE(A_GPS_UART_EN, IO_E_A); 
    else OS_Yield();

    /******************************************************************************
    * Transmit new data 
    ******************************************************************************/  
    //UART_Channel_Select(STAMP_MSP_CONNECT);

    /******************************************************************************
    * Signal new nmea string to be logged 
    ******************************************************************************/  
    if (!e.gps_r) OSSignalBinSem(LOG_LOW_F_P);
     /******************************************************************************
    * Update low frequency time 
    ******************************************************************************/  
    data.LF_t_elap = update_time( &(data.h_LF) , &(data.m_LF), &(data.s_LF), &(data.new_gps_LF));  
    OS_Yield();
    }
 }