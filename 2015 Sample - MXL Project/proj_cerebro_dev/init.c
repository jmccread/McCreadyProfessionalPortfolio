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
 * date:	07/23/2014
 */

/************************************************************ 
* Includes
************************************************************/
#include "init.h"

/************************************************************ 
* Declarations
************************************************************/


/************************************************************ 
*
* init.c
*
************************************************************/
void init(void) {

  
  // Force WDT off for now ... having it on tends to confuse novice
  //   users.
  csk_wdt_off();
  
  // Keep interrupts off for now ...
  __disable_interrupt();
  
  // All CSK control signals are active LOW.
  P1OUT  =  0xFF;
  P2OUT  =  0xFF;
  P3OUT  =  0xFF;
  P4OUT  =  0xFF;
  P5OUT  =  0XFF;
  P6OUT  =  0xFF;
  
  // initialize all the error messages to 0
  e.i2c = 0; 
  e.gyro = 0; 
  e.acc = 0;
  e.adc = 0; 
  e.mag = 0; 
  e.iso = 0; 
  e.pca = 0; 
  e.tca = 0; 
  e.bmp = 0; 
  
  // High-level inits (works at any clock speed).
  csk_sd_close();
  csk_sd_pwr_off();
  
 
  // Configure Clocks, speeds can be found in config.h 
  DCOCTL=0xE0;                              // Configure DCO to run at 20 MHz DCO = 7
  BCSCTL1=0x8F;                              // Select DCO for MCLK, Configure RSEL 7
  BCSCTL2 = 0x00;                           // No division of SMCLK
  BCSCTL3 = (XCAP_2|LFXT1OF)|0x00;          // Set fault condition and capacitance (TO DO: Check cap)

  // Configure Timer A
  TACTL  |=  BIT8 + BIT4;                   //  Select ACLK, Start Timer_A in Up Mode  
  CCTL0  = CCIE;                            // CCR0 interrupt enabled
  TACCR0   = 33; 
  data.ms_counter = 0; 

  // Initialize Isolation on the UART and I2C Buses 
  I2C_SEL |= (PIN_SDA+PIN_SCL);    
  #ifdef I2C_UCB0
  I2C_Init_Master(USCI_B_I2C_CLOCKSOURCE_SMCLK,
                 SMCLK_SPD,
                 USCI_B_I2C_SET_DATA_RATE_400KBPS);
  #endif

   __enable_interrupt();
  init_isolation_state();  
  
}
/************************************************************ 
*
* delay function 
*
************************************************************/
void delay_ms(uint16_t delay_ms) {
  uint16_t start =  data.ms_counter;
  uint16_t t_left = 60000- start; 
  if (delay_ms< t_left) 
    {
    while(data.ms_counter < (start+delay_ms)); 
    }
  else if (delay_ms> t_left)
    {
    while (!(data.ms_counter == (delay_ms-t_left)));
    } 
  }

/******************************************************************************
*
isr_timerA0() 1 ms ticks 
*
******************************************************************************/
void isr_timerA0(void) __interrupt[TIMERA0_VECTOR] {
  OSTimer();
  
  if (data.ms_counter == 60000)
  {
    data.ms_counter = 0; 
  }
  // Create a 30 second tick for other timing applications 
  if ( (data.ms_counter == 2*29999)|(data.ms_counter == 29999))
  //if (!(data.ms_counter%5000))
  {
    OSSignalBinSem(COLLECT_LOW_F_P); 
    OSSignalBinSem(TEMP_CHECK_P); 
  } 
  data.ms_counter++; 
  data.tick_counter++; 
}






