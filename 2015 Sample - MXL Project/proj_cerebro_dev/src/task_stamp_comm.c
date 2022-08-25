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
* task_stamp_comm()
*
******************************************************************************/

/******************************************************************************
* Includes 
******************************************************************************/
#include "task_stamp_comm.h"
#define TIMEOUT 0.25   //seconds 
/******************************************************************************
* Globals 
******************************************************************************/


void task_stamp_comm(void) {

  /******************************************************************************
  * Setup
  ******************************************************************************/
  uint8_t cmd_rxd = 0; 
  uint8_t cmd_tx = 0xFF; 
  while(1) 
    {
    /******************************************************************************
    * Wait for new measurement available, isolate UART devices then change the UART 
    * channel to allow for MSP to Stamp communications 
    ******************************************************************************/ 
    OS_WaitBinSem(NEW_HIGH_F_P, OSNO_TIMEOUT);
    UART_Channel_Select(STAMP_MSP_CONNECT);

    csk_led_status_open(PIN_LED_1);  
    /******************************************************************************
    * Configure to do fast UART and transmit data
    ******************************************************************************/
    //uart_transmit_float(&UART_A1,  &test, 0, 1);
    csk_led_status_off(PIN_LED_1);
    start_device_uart(460800);
    uart_transmit_float(&UART_A1,  &data.gyro_dps, GYRO_IDENT,  3);
    uart_transmit_float(&UART_A1,  &data.acc_g, ACC_IDENT,   3);
    uart_transmit_float(&UART_A1,  &data.photo_V,  PHOTO_IDENT, 15);
    uart_transmit_float(&UART_A1,  &data.mag_g, 0x03, 12);
    csk_led_status_on(PIN_LED_1);
   

    /******************************************************************************
    * In the event that you want to make the linux machine ack that it recieved you could 
    ******************************************************************************/
   /*
    //data.COM_t_elap = update_time(&(data.h_COM) , &(data.m_COM), &(data.s_COM), &(data.new_gps_COM));  
    while ((!cmd_rxd) && (update_time( &(data.h_COM) , &(data.m_COM), &(data.s_COM), &(data.new_gps_COM)) < (data.COM_t_elap+TIMEOUT)))
    {
      //uart_transmit_uint8(&UART_A1,21 &cmd_tx, 0x09, 1); 
      if (uart_recieve_uint8(&UART_A1, MCLK_SPD , &cmd_rxd, 0x08))
    }
    cmd_rxd = 0; */

    OS_Yield();
    }
 }