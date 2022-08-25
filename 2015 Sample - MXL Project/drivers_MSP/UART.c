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
* UART.c
******************************************************************************/

/********************************************************************************
*
* Includes
*
 *******************************************************************************/
#include "UART.h"
#include "msp430.h"
/********************************************************************************
*
* Definitions
*
 *******************************************************************************/
// Global Variables 
rbuffer_t UART_RX;
rbuffer_t UART_TX; 
usci_a_uart_t UART_A1;
uint8_t got_byte, send_byte;
uint16_t i=0; 

/*************************************************************************************
 * Function:          void start_device_uart(void)
 *
 * PreCondition:   
 *
 * Input:           
 *
 * Output:           
 *
 * Side Effects:    
 *
 * Overview:      
 *
 * Note:          
 *                  
 *************************************************************************************/
uint8_t start_device_uart(uint32_t baud_rate ) {
  uint8_t error;
  // Enable UART functionality (TO DO: Elegant clock)
  __enable_interrupt();
   error = USCI_A_UART_init (&UART_A1,
                    USCI_A1_BASE,
                    USCI_A_UART_CLOCKSOURCE_SMCLK, 
                    SMCLK_SPD,
                    baud_rate,
                    USCI_A_UART_NO_PARITY,
                    USCI_A_UART_LSB_FIRST,
                    USCI_A_UART_ONE_STOP_BIT,
                    USCI_A_UART_MODE,
                    USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION);   
                  
  if (error == STATUS_FAIL) 
    return 0; 

  //enable UART Pins (TO DO: Change?)
  USCI_A_UART_enable (&UART_A1);
  P3SEL |= BIT6 + BIT7;
  P3DIR |= BIT6;
  
  //Enable UART interrupts 
  USCI_A_UART_enableInterrupt (&UART_A1, USCI_A_UART_RECEIVE_INTERRUPT);
  USCI_A_UART_enableInterrupt (&UART_A1, USCI_A_UART_TRANSMIT_INTERRUPT);

  }


/*************************************************************************************
 * Function:        uint8_t Wait_for_UART_Not_Busy(usci_a_uart_t *in_ptr, uint32_t master_speed)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Waits for the UART USCI to be ready, lets the system hang for ~1 
 *                  second after setting the Master clock speed (assuming that that counting
 *                  counting up to the clock speed would be a second. Returns 1 if busy
 *
 * Note:            This function should not be used on its own but as part of a write
 *                  or read operation on the UART bus
*************************************************************************************/
uint8_t Wait_for_UART_Not_Busy(usci_a_uart_t *in_ptr, uint32_t master_speed)
{
  uint32_t ui32_UART_counter = 0;             // I2C busy counter             
  uint8_t debug =0 ; 

  //Wait for UART to not be busy.  If it takes too long return an error 
  for(ui32_UART_counter = 0; ui32_UART_counter < master_speed ; ui32_UART_counter++){
    // should return 1 if busy, if !1, only breaks if not busy
    if (!(USCI_A_UART_queryStatusFlags(in_ptr, USCI_A_UART_BUSY)))
      {
      return UART_NO_ERROR;
      }
   }
  
  // Throw error once the counter overflows. 
  return ERROR_UART_BUSY;
}

/*************************************************************************************
 * Function:        uint8_t uart_transmit_float(usci_a_uart_t *in_ptr,
 *                                              float * d, uint8_t ID, uint8_t length)
 *
 * PreCondition:   start_device_uart has run
 *
 * Input:           Parameters
 *                  - usci_a_uart_t * UART_S -- uart struct being used (USCI_A0/A1)
 *                  - float * d -- pointer to float datums to be sent via uart
 *                  - uint8_t ID -- Extra identifier byte provided by a user
 *                  - uint8_t length -- number floating point datums to be sent
 *                    -- MUST BE less than 126! 
 *
 * Output:          if successful returns 1, if not returns 0
 *
 * Side Effects:    Writes d over USCI A1 via uart according to the configuration in 
 *                  start_device_uart
 *
 * Overview:        Puts bytes in the UART_TX ring buffer according to the simple protocol 
 *                  defined in UART.h and adds a fletcher16 checksum 
 *
 * Note:            The maximum size for a ring buffer is 512, which means that a maximum of 
 *                  of 126 32 bit floating point numbers can be sent (7 bytes of the ring 
 *                  ring buffer are non-negotiable protocol bytes)
 *                  
 *************************************************************************************/
uint8_t uart_transmit_float(usci_a_uart_t *in_ptr, float * d, uint8_t ID, uint8_t length)
{
  // Variables
  uint8_t buff[4];
  uint8_t cs_buff[512];
  uint8_t checksum[2]; 
  uint8_t e, i;
  
  // Clear the transmit buffer 
  rbuffer_clear(&UART_TX);

  // Load uart start sequence
  rbuffer_putchar(&UART_TX, UART_START_BYTE);
  rbuffer_putchar(&UART_TX, UART_FLOAT);
  rbuffer_putchar(&UART_TX, ID);
  rbuffer_putchar(&UART_TX, length); 

  // Load all the floats 
  for (i=0; i<length; i++) {
    // Most significant byte at the end of the buffer
    memcpy(buff, &d[i], 4);
    rbuffer_putchars(&UART_TX, buff, 4);
    }
  // End d 
  rbuffer_putchar(&UART_TX, UART_STOP_BYTE); 
  // Calculate two byte checksum;
  calculate_checksum(checksum, UART_TX.buff, rbuffer_count(&UART_TX));
  rbuffer_putchars(&UART_TX, checksum, 2);

  // Send d 
  USCI_A_UART_transmitData(in_ptr, (rbuffer_getchar(&UART_TX)));

  // Wait until tx chars have been drained; 
  while(rbuffer_count(&UART_TX));
  return 1; 
}


/*************************************************************************************
 * Function:        uint8_t uart_transmit_uint32(usci_a_uart_t *in_ptr,  
 *                                                uint32_t * d, uint8_t ID, uint8_t length)
 *
 * PreCondition:   start_device_uart has run
 *
 * Input:           Parameters
 *                  - usci_a_uart_t * UART_S -- uart struct being used (USCI_A0/A1)
 *                  - float * d -- pointer to uint32_t datums to be sent via uart
 *                  - uint8_t ID -- Extra identifier byte provided by a user
 *                  - uint8_t length -- unsigned 32 bit datums to be sent
 *                    -- MUST BE less than 126! 
 *
 * Output:          if successful returns 1, if not returns 0
 *
 * Side Effects:    Writes d over USCI A1 via uart according to the configuration in 
 *                  start_device_uart
 *
 * Overview:        Puts bytes in the UART_TX ring buffer according to the simple protocol 
 *                  defined in UART.h and adds a fletcher16 checksum 
 *
 * Note:            The maximum size for a ring buffer is 512, which means that a maximum of 
 *                  of 126 32 bit numbers can be sent (7 bytes of the ring 
 *                  ring buffer are non-negotiable protocol bytes)
 *                  
 *************************************************************************************/
uint8_t uart_transmit_uint32(usci_a_uart_t *in_ptr,  
                               uint32_t * d, uint8_t ID, uint8_t length)
{
  // Variables
  uint8_t buff[sizeof(uint32_t)]; 
  uint8_t checksum[2]; 
  uint8_t e, i;
  
  // Clear the transmit buffer 
  rbuffer_clear(&UART_TX);

  // Load uart start sequence
  
  rbuffer_putchar(&UART_TX, UART_START_BYTE);
  rbuffer_putchar(&UART_TX, UART_UINT32);
  rbuffer_putchar(&UART_TX, ID);
  rbuffer_putchar(&UART_TX, length);

  // Load all the floats 
  for (i=0; i<length; i++) {
    // Most Significant byte sent first
    memcpy(buff, &d[i], sizeof(uint32_t));
    // load buffer into ring buffer
    rbuffer_putchars(&UART_TX, buff, 4);
    }
  // End d 
  rbuffer_putchar(&UART_TX, UART_STOP_BYTE);
  
  // Calculate two byte checksum
  calculate_checksum(checksum, UART_TX.buff, rbuffer_count(&UART_TX));
  rbuffer_putchars(&UART_TX, checksum, 2);
 
  // Send d 
  USCI_A_UART_transmitData(in_ptr, (rbuffer_getchar(&UART_TX)));
  
  // Wait until tx chars have been drained; 
  while(rbuffer_count(&UART_TX)); 
  return 1; 
}

/*************************************************************************************
 * Function:       uint8_t uart_transmit_int32(usci_a_uart_t *in_ptr,  
 *                                            int32_t * d, uint8_t ID,  uint8_t length)
 * PreCondition:   start_device_uart has run
 *
 * Input:           Parameters
 *                  - usci_a_uart_t * UART_S -- uart struct being used (USCI_A0/A1)
 *                  - float * d -- pointer to float datums to be sent via uart
 *                  - uint8_t ID -- Extra identifier byte provided by a user
 *                  - uint8_t length -- number floating point datums to be sent
 *                    -- MUST BE less than 126! 
 *
 * Output:          if successful returns 1, if not returns 0
 *
 * Side Effects:    Writes d over USCI A1 via uart according to the configuration in 
 *                  start_device_uart
 *
 * Overview:        Puts bytes in the UART_TX ring buffer according to the simple protocol 
 *                  defined in UART.h and adds a fletcher16 checksum 
 *
 * Note:            The maximum size for a ring buffer is 512, which means that a maximum of 
 *                  of 126 32 bit floating point numbers can be sent (7 bytes of the ring 
 *                  ring buffer are non-negotiable protocol bytes)
 *                  
 *************************************************************************************/
uint8_t uart_transmit_int32(usci_a_uart_t *in_ptr,  
                                   int32_t * d, uint8_t ID,  uint8_t length)
{
  // Variables
  uint8_t buff[sizeof(int32_t)]; 
  uint8_t checksum[2];  
  uint8_t e, i;
  
  // Clear the transmit buffer 
  rbuffer_clear(&UART_TX);

  // Load uart start sequence
  
  rbuffer_putchar(&UART_TX, UART_START_BYTE);
  rbuffer_putchar(&UART_TX, UART_INT32);
  rbuffer_putchar(&UART_TX, ID);
  rbuffer_putchar(&UART_TX, length);
  // Load all the d 
  for (i=0; i<length; i++) {
    memcpy(buff, &d[i], sizeof(int32_t));
    // load buffer into ring buffer
    rbuffer_putchars(&UART_TX, buff, 4);
    }
  // End d 
  rbuffer_putchar(&UART_TX, UART_STOP_BYTE);
  
  // Calculate two byte checksum
  calculate_checksum(checksum, UART_TX.buff, rbuffer_count(&UART_TX));
  rbuffer_putchars(&UART_TX, checksum, 2);
 
  // Send d 
  USCI_A_UART_transmitData(in_ptr, (rbuffer_getchar(&UART_TX)));
  
  // Wait until tx chars have been drained; 
  while(rbuffer_count(&UART_TX));
  return 1; 
}

/*************************************************************************************
 * Function:        uint8_t uart_transmit_int16(usci_a_uart_t *in_ptr,  
 *                                               int16_t * d, uint8_t length) 
 *
 * PreCondition:   start_device_uart has run
 *
 * Input:           Parameters
 *                  - usci_a_uart_t * UART_S -- uart struct being used (USCI_A0/A1)
 *                  - float * d -- pointer to int16 datums to be sent via uart
 *                  - uint8_t ID -- Extra identifier byte provided by a user
 *                  - uint8_t length -- number of int16 datums to be sent
 *                    -- MUST BE less than 252! 
 *
 * Output:          if successful returns 1, if not returns 0
 *
 * Side Effects:    Writes d over USCI A1 via uart according to the configuration in 
 *                  start_device_uart
 *
 * Overview:        Puts bytes in the UART_TX ring buffer according to the simple protocol 
 *                  defined in UART.h and adds a fletcher16 checksum 
 *./
 * Note:            The maximum size for a ring buffer is 512, which means that a maximum of 
 *                  of 252 16 bit numbers can be sent (7 bytes of the ring 
 *                  ring buffer are non-negotiable protocol bytes)
 *                  
 *************************************************************************************/
uint8_t uart_transmit_int16(usci_a_uart_t *in_ptr, int16_t * d, 
                            uint8_t ID, uint8_t length)
{
  // Variables
  uint8_t buff[sizeof(int16_t)]; 
  uint8_t checksum[2]; 
  uint16_t temp; 
  uint8_t e, i;

  // Clear the transmit buffer 
  rbuffer_clear(&UART_TX);

  // Load uart start sequence
  rbuffer_putchar(&UART_TX, UART_START_BYTE);
  rbuffer_putchar(&UART_TX, UART_INT16);
  rbuffer_putchar(&UART_TX, ID);
  rbuffer_putchar(&UART_TX, length);

  // Load all the d
  for (i=0; i<length; i++) {
    // Most Significant byte in last position 
    memcpy(buff, &d[i], sizeof(int16_t));
    rbuffer_putchars(&UART_TX, buff, sizeof(int16_t));
    }
  // End d 
  rbuffer_putchar(&UART_TX, UART_STOP_BYTE);
  
  // Calculate two byte checksum
  calculate_checksum(checksum, UART_TX.buff, rbuffer_count(&UART_TX));
  rbuffer_putchars(&UART_TX, checksum, 2);
  
  // Send d 
  USCI_A_UART_transmitData(in_ptr, (rbuffer_getchar(&UART_TX)));
  
  // Wait until tx chars have been drained; 
  while(rbuffer_count(&UART_TX));
  return 1; 
}

/*************************************************************************************
 * Function:        uint8_t uart_transmit_uint16(usci_a_uart_t *in_ptr,  
 *                                               uint16_t * d, uint8_t length) 
 *
 * PreCondition:   start_device_uart has run
 *
 * Input:           Parameters
 *                  - usci_a_uart_t * UART_S -- uart struct being used (USCI_A0/A1)
 *                  - float * d -- pointer to int16 datums to be sent via uart
 *                  - uint8_t ID -- Extra identifier byte provided by a user
 *                  - uint8_t length -- number of int16 datums to be sent
 *                    -- MUST BE less than 252! 
 *
 * Output:          if successful returns 1, if not returns 0
 *
 * Side Effects:    Writes d over USCI A1 via uart according to the configuration in 
 *                  start_device_uart
 *
 * Overview:        Puts bytes in the UART_TX ring buffer according to the simple protocol 
 *                  defined in UART.h and adds a fletcher16 checksum 
 *
 * Note:            The maximum size for a ring buffer is 512, which means that a maximum of 
 *                  of 252 16 bit numbers can be sent (7 bytes of the ring 
 *                  ring buffer are non-negotiable protocol bytes)
 *                  
 *************************************************************************************/
uint8_t uart_transmit_uint16(usci_a_uart_t *in_ptr,uint16_t * d,
                             uint8_t ID, uint8_t length)
{
  // Variables
  uint8_t buff[2]; 
  uint8_t checksum[2];  
  uint8_t e, i;
  
  // Clear the transmit buffer 
  rbuffer_clear(&UART_TX);

  // Load uart start sequence
  rbuffer_putchar(&UART_TX, UART_START_BYTE);
  rbuffer_putchar(&UART_TX, UART_UINT16);
  rbuffer_putchar(&UART_TX, ID);
  rbuffer_putchar(&UART_TX, length);

  // Load all the floats 
  for (i=0; i<length; i++) {
    // Most Significant byte sent first;
    memcpy(buff, &d[i], sizeof(uint16_t));
    // load buffer into ring buffer
    rbuffer_putchars(&UART_TX, buff, sizeof(uint16_t));
    }
  // End d 
  rbuffer_putchar(&UART_TX, UART_STOP_BYTE);
  
  // Calculate two byte checksum
  calculate_checksum(checksum, UART_TX.buff, rbuffer_count(&UART_TX));
  rbuffer_putchars(&UART_TX, checksum, 2);

  // Send d 
  USCI_A_UART_transmitData(in_ptr, (rbuffer_getchar(&UART_TX)));
  
  // Wait until tx chars have been drained; 
  while(rbuffer_count(&UART_TX));
  return 1;  
}

/*************************************************************************************
 * Function:       uint8_t uart_transmit_uint8(usci_a_uart_t *in_ptr, uint8_t * d, 
 *                           uint8_t ID, uint8_t length)
 *
 * PreCondition:   start_device_uart has run
 *
 * Input:           Parameters
 *                  - usci_a_uart_t * UART_S -- uart struct being used (USCI_A0/A1)
 *                  - float * d -- pointer to int16 datums to be sent via uart
 *                  - uint8_t ID -- Extra identifier byte provided by a user
 *                  - uint8_t length -- number of int16 datums to be sent
 *                    -- MUST BE less than 252! 
 *
 * Output:          if successful returns 1, if not returns 0
 *
 * Side Effects:    Writes d over USCI A1 via uart according to the configuration in 
 *                  start_device_uart
 *
 * Overview:        Puts bytes in the UART_TX ring buffer according to the simple protocol 
 *                  defined in UART.h and adds a fletcher16 checksum 
 *
 * Note:            The maximum size for a ring buffer is 512, which means that a maximum of 
 *                  of 504 8 bit numbers can be sent (7 bytes of the ring 
 *                  ring buffer are non-negotiable protocol bytes)
 *                  
 *************************************************************************************/
uint8_t uart_transmit_uint8(usci_a_uart_t *in_ptr, uint8_t * d, 
                            uint8_t ID, uint8_t length)
{
  // Variables
  uint8_t checksum[2];  
  uint8_t e, i;
  
  // Clear the transmit buffer 
  rbuffer_clear(&UART_TX);

  // Load uart start sequence
  rbuffer_putchar(&UART_TX, UART_START_BYTE);
  rbuffer_putchar(&UART_TX, UART_UINT8);
  rbuffer_putchar(&UART_TX, ID);
  rbuffer_putchar(&UART_TX, length);

  // Load all the d
  for (i=0; i<length; i++) {
    // load d into ring buffer
    rbuffer_putchar(&UART_TX, d[i]);
    }
  // End d 
  rbuffer_putchar(&UART_TX, UART_STOP_BYTE);
  
  // Calculate two byte checksum
  calculate_checksum(checksum, UART_TX.buff, rbuffer_count(&UART_TX));
  rbuffer_putchars(&UART_TX, checksum, 2);
  
  // Send d 
  USCI_A_UART_transmitData(in_ptr, (rbuffer_getchar(&UART_TX)));
  
  // Wait until tx chars have been drained; 
  while(rbuffer_count(&UART_TX));
  return 1; 
}

/*************************************************************************************
 * Function:        int8_t uart_transmit_uint8(usci_a_uart_t *in_ptr,  
 *                                               uint8_t * d, uint8_t length) 
 *
 * PreCondition:   start_device_uart has run
 *
 * Input:           Parameters
 *                  - usci_a_uart_t * UART_S -- uart struct being used (USCI_A0/A1)
 *                  - float * d -- pointer to int16 datums to be sent via uart

 *                  - uint8_t length -- number of int16 datums to be sent
 *                    -- MUST BE less than 252! 
 *
 * Output:          if successful returns 1, if not returns 0
 *
 * Side Effects:    Writes d over USCI A1 via uart according to the configuration in 
 *                  start_device_uart
 *
 * Overview:        Puts bytes in the UART_TX ring buffer according to the simple protocol 
 *                  defined in UART.h and adds a fletcher16 checksum 
 *
 * Note:            The maximum size for a ring buffer is 512, which means that a maximum of 
 *                  of 504 8 bit numbers can be sent (7 bytes of the ring 
 *                  ring buffer are non-negotiable protocol bytes)
 *                  
 *************************************************************************************/
uint8_t uart_transmit_int8(usci_a_uart_t *in_ptr, int8_t * d, uint8_t ID, uint8_t length)
{
  // Variables
  uint8_t checksum[2];  
  uint8_t e, i;
  
  // Clear the transmit buffer 
  rbuffer_clear(&UART_TX);

  // Load uart start sequence
  rbuffer_putchar(&UART_TX, UART_START_BYTE);
  rbuffer_putchar(&UART_TX, UART_INT8);
  rbuffer_putchar(&UART_TX, ID);
  rbuffer_putchar(&UART_TX, length);

  // Load all the d
  for (i=0; i<length; i++) {
    // load d into ring buffer
    rbuffer_putchar(&UART_TX, (uint8_t)d[i]);
    }
  // End d 
  rbuffer_putchar(&UART_TX, UART_STOP_BYTE);
  
  // Calculate two byte checksum
  calculate_checksum(checksum, UART_TX.buff, rbuffer_count(&UART_TX));
  rbuffer_putchars(&UART_TX, checksum, 2);
  
  // Send d 
  USCI_A_UART_transmitData(in_ptr, (rbuffer_getchar(&UART_TX)));
  
  // Wait until tx chars have been drained; 
  while(rbuffer_count(&UART_TX));
  return 1; 
}

/*************************************************************************************
 * Function:          uart_recieve_float(usci_a_uart_t *in_ptr, uint32_t mclk_speed, float * d, uint8_t ID) 
 *
 * PreCondition:      1. start_device_uart has run
 *                    2. UART d has been sent and has been recieved by msp430
 *
 * Input:             Parameters:
 *                    - usci_a_uart_t *in_ptr: Uart A bus being used
 *                    - uint32_t mclk_speed: essentially a timeout for the wait uart busy fn.
 *                    - float * d: Pointer to where the received data is going 
 *                    - uint8_t ID: Application specific identifier
 *  
 * Output:            1 if UART has correctly been received according to 'protocol' in UART.h
 *                    0 if something broke... 
 *
 * Side Effects:      Writes to float * d 
 *  
 * Overview:          1. Waits for UART_Not_Busy to run with timeout
 *                    2. Cycles through ring buffer where recieved uart bytes are stored
 *                    3. Checks bytes and checksum, returns 0 if something didn't match
 *
 * Note:              None 
 *                  
 *************************************************************************************/
uint8_t uart_recieve_float(usci_a_uart_t *in_ptr, uint32_t mclk_speed, float * d, uint8_t ID) {
  
  // Variables 
  uint8_t buff, length, RXd_CS[2], CALCd_CS[2], cs_buff[512], temp[sizeof(float)]; 
  uint8_t i=0;
  uint8_t j=0;
  uint16_t k = 0;
  volatile int state=0;

  // Wait for all UART transmissions to end
  if (Wait_for_UART_Not_Busy(in_ptr, mclk_speed)) 
    return 0; 
  // Once done start parsing d
  else
  {
    while(rbuffer_count(&UART_RX))
    {
     buff= rbuffer_getchar(&UART_RX);
     cs_buff[k] = buff;
     k++;
     switch(state) 
      {
      // Cases 0 - 2 are tell the type a
      case 0 :
        if (buff == UART_START_BYTE) state = 1;
        else return 0 ;
        break;
      case 1 : 
        if (buff == UART_FLOAT) state = 2; 
        else return 0;
        break;
      case 2 : 
        ID = buff; 
        state = 3; 
      case 3 : 
        length = buff; 
        state =4; 
        break; 
      // Get all the individual bytes to assemble into a d piece 
      case 4 : 
         switch(i) {
          case 0: 
            temp[i] = buff;
            i++; 
            break; 
          case 1: 
            temp[i] = buff;
            i++; 
            break;
          case 2: 
            temp[i]= buff;
            i++;
            break;
          case 3: 
            temp[i] = buff;
            i = 0;
            memcpy(&d[j], temp, sizeof(float));
            j++; 
            if (j == (length)) state = 5; 
            break;
          }
        break;
      // Check for stop byte
      case 5: 
        if (buff == UART_STOP_BYTE) state = 6;  
        else return 0 ; 
        break; 
      // Get the checksum then check it. 
      case 6: 
        RXd_CS[0] = buff; state= 7; 
        break;
      case 7:
        RXd_CS[1] = buff; state=8; 
        calculate_checksum(CALCd_CS, cs_buff, length*sizeof(float)+4);
        // Check the calculated and received checksum
        if ((CALCd_CS[0]==RXd_CS[0]) && (CALCd_CS[1]==RXd_CS[1]))  
          return 1;       // IF they match return 1
        else 
          return 0; 
        break;
      default: 
        state = 0; 
      }
    }  
   }
   } /*Uart_Recieve_float()*/

/*************************************************************************************
 * Function:          uint8_t uart_recieve_uint32(usci_a_uart_t *in_ptr, 
 *                                               uint32_t mclk_speed, uint32_t * d, uint8_t ID)
 *
 * PreCondition:      1. start_device_uart has run
 *                    2. UART d has been sent and has been recieved by msp430
 *
 * Input:             Parameters:
 *                    - usci_a_uart_t *in_ptr: Uart A bus being used
 *                    - uint32_t mclk_speed: essentially a timeout for the wait uart busy fn.
 *                    - uint32_t * d: Pointer to where the received data is going 
 *                    - uint8_t ID: Application specific identifier
 *  
 * Output:            1 if UART has correctly been received according to 'protocol' in UART.h
 *                    0 if something broke... 
 *
 * Side Effects:      Writes to uint32_t * d 
 *  
 * Overview:          1. Waits for UART_Not_Busy to run with timeout
 *                    2. Cycles through ring buffer where recieved uart bytes are stored
 *                    3. Checks bytes and checksum, returns 0 if something didn't match
 *
 * Note:              None 
 *                  
 *************************************************************************************/
uint8_t uart_recieve_uint32(usci_a_uart_t *in_ptr, uint32_t mclk_speed, uint32_t * d, uint8_t ID) {
  
  // Variables 
  uint8_t buff, length, RXd_CS[2], CALCd_CS[2], cs_buff[512], temp[sizeof(uint32_t)]; 
  uint8_t i=0;
  uint8_t j=0;
  uint16_t k = 0;
  volatile int state=0;

  // Wait for all UART transmissions to end
  if (Wait_for_UART_Not_Busy(in_ptr, mclk_speed)) 
    return 0; 
  // Once done start parsing d
  else
  {
    while(rbuffer_count(&UART_RX))
    {
     buff= rbuffer_getchar(&UART_RX);
     cs_buff[k] = buff;
     k++;
     switch(state) 
      {
      // Cases 0 - 2 are tell the type a
      case 0 :
        if (buff == UART_START_BYTE) state = 1;
        else return 0 ;
        break;
      case 1 : 
        if (buff == UART_UINT32) state = 2; 
        else return 0;
        break;
      case 2 : 
        ID = buff; 
        state = 3; 
      case 3 : 
        length = buff; 
        state = 4; 
        break; 
      // Get all the individual bytes to assemble into a d piece 
      case 4 : 
         switch(i) {
          case 0: 
            temp[i] = buff;
            i++; 
            break; 
          case 1: 
            temp[i] = buff;
            i++; 
            break;
          case 2: 
            temp[i]= buff;
            i++;
            break;
          case 3: 
            temp[i] = buff;
            i = 0;
            memcpy(&d[j], temp, sizeof(float));
            j++; 
            if (j == (length)) state = 5; 
            break;
          }
        break;
      // Check for stop byte
      case 5: 
        if (buff == UART_STOP_BYTE) state = 6;  
        else return 0 ; 
        break; 
      // Get the checksum then check it. 
      case 6: 
        RXd_CS[0] = buff; state= 7; 
        break;
      case 7:
        RXd_CS[1] = buff; state=8; 
        calculate_checksum(CALCd_CS, cs_buff, length*sizeof(uint32_t)+4);
        // Check the calculated and received checksum
        if ((CALCd_CS[0]==RXd_CS[0]) && (CALCd_CS[1]==RXd_CS[1]))  
          return 1;       // IF they match return 1
        else 
          return 0; 
        break;
      default: 
        state = 0; 
      }
    }  
   }
   } /*uart_recieve_uint32 */

/*************************************************************************************
 * Function:          uint8_t uart_recieve_int32(usci_a_uart_t *in_ptr, 
 *                                               uint32_t mclk_speed, int32_t * d, uint8_t ID)
 *
 * PreCondition:      1. start_device_uart has run
 *                    2. UART d has been sent and has been recieved by msp430
 *
 * Input:             Parameters:
 *                    - usci_a_uart_t *in_ptr: Uart A bus being used
 *                    - uint32_t mclk_speed: essentially a timeout for the wait uart busy fn.
 *                    - int32_t * d: Pointer to where the received data is going 
 *                    - uint8_t ID: Application specific identifier
 *  
 * Output:            1 if UART has correctly been received according to 'protocol' in UART.h
 *                    0 if something broke... 
 *
 * Side Effects:      Writes to int32_t * d 
 *  
 * Overview:          1. Waits for UART_Not_Busy to run with timeout
 *                    2. Cycles through ring buffer where recieved uart bytes are stored
 *                    3. Checks bytes and checksum, returns 0 if something didn't match
 *
 * Note:              None 
 *                  
 *************************************************************************************/
uint8_t uart_recieve_int32(usci_a_uart_t *in_ptr, uint32_t mclk_speed, int32_t * d, uint8_t ID) {
  
  // Variables 
  uint8_t buff, length, RXd_CS[2], CALCd_CS[2], cs_buff[512], temp[sizeof(int32_t)]; 
  uint8_t i=0;
  uint8_t j=0;
  uint16_t k = 0;
  volatile int state=0;

  // Wait for all UART transmissions to end
  if (Wait_for_UART_Not_Busy(in_ptr, mclk_speed)) 
    return 0; 
  // Once done start parsing d
  else
  {
    while(rbuffer_count(&UART_RX))
    {
     buff= rbuffer_getchar(&UART_RX);
     cs_buff[k] = buff;
     k++;
     switch(state) 
      {
      // Cases 0 - 2 are tell the type a
      case 0 :
        if (buff == UART_START_BYTE) state = 1;
        else return 0 ;
        break;
      case 1 : 
        if (buff == UART_INT32) state = 2; 
        else return 0;
        break;
      case 2 : 
        ID = buff; 
        state = 3; 
      case 3 : 
        length = buff; 
        state =4; 
        break; 
      // Get all the individual bytes to assemble into a d piece 
      case 4 : 
         switch(i) {
          case 0: 
            temp[i] = buff;
            i++; 
            break; 
          case 1: 
            temp[i] = buff;
            i++; 
            break;
          case 2: 
            temp[i]= buff;
            i++;
            break;
          case 3: 
            temp[i] = buff;
            i = 0;
            memcpy(&d[j], temp, sizeof(float));
            j++; 
            if (j == (length)) state = 5; 
            break;
          }
        break;
      // Check for stop byte
      case 5: 
        if (buff == UART_STOP_BYTE) state = 6;  
        else return 0 ; 
        break; 
      // Get the checksum then check it. 
      case 6: 
        RXd_CS[0] = buff; state= 7; 
        break;
      case 7:
        RXd_CS[1] = buff; state=8; 
        calculate_checksum(CALCd_CS, cs_buff, length*sizeof(int32_t)+4);
        // Check the calculated and received checksum
        if ((CALCd_CS[0]==RXd_CS[0]) && (CALCd_CS[1]==RXd_CS[1]))  
          return 1;       // IF they match return 1
        else 
          return 0; 
        break;
      default: 
        state = 0; 
      }
    }  
   }
 } /*uart_recieve_int32 */

/*************************************************************************************
 * Function:          uint8_t uart_recieve_int16(usci_a_uart_t *in_ptr, 
 *                                               uint16_t mclk_speed, int16_t * d, uint8_t ID)
 *
 * PreCondition:      1. start_device_uart has run
 *                    2. UART d has been sent and has been recieved by msp430
 *
 * Input:             Parameters:
 *                    - usci_a_uart_t *in_ptr: Uart A bus being used
 *                    - uint32_t mclk_speed: essentially a timeout for the wait uart busy fn.
 *                    - int16_t * d: Pointer to where the received data is going 
 *                    - uint8_t ID: Application specific identifier
 *  
 * Output:            1 if UART has correctly been received according to 'protocol' in UART.h
 *                    0 if something broke... 
 *
 * Side Effects:      Writes to uint16_t * d 
 *  
 * Overview:          1. Waits for UART_Not_Busy to run with timeout
 *                    2. Cycles through ring buffer where recieved uart bytes are stored
 *                    3. Checks bytes and checksum, returns 0 if something didn't match
 *
 * Note:              None 
 *                  
 *************************************************************************************/
uint8_t uart_recieve_uint16(usci_a_uart_t *in_ptr, uint32_t mclk_speed, uint16_t * d, uint8_t ID) {
  
  // Variables 
  uint8_t buff, length, RXd_CS[2], CALCd_CS[2], cs_buff[512], temp[sizeof(uint16_t)]; 
  uint8_t i=0;
  uint8_t j=0;
  uint16_t k = 0;
  volatile int state=0;

  // Wait for all UART transmissions to end
  if (Wait_for_UART_Not_Busy(in_ptr, mclk_speed)) 
    return 0; 
  // Once done start parsing d
  else
  {
    while(rbuffer_count(&UART_RX))
    {
     buff= rbuffer_getchar(&UART_RX);
     cs_buff[k] = buff;
     k++;
     switch(state) 
      {
      // Cases 0 - 2 are tell the type a
      case 0 :
        if (buff == UART_START_BYTE) state = 1;
        else return 0 ;
        break;
      case 1 : 
        if (buff == UART_UINT16) state = 2; 
        else return 0;
        break;
      case 2 : 
        ID = buff; 
        state = 3;       
      case 3 : 
        length = buff; 
        state =4; 
        break; 
      // Get all the individual bytes to assemble into a d piece 
      case 4 : 
         switch(i) {
          case 0: 
            temp[i]= buff;
            i++;
            break;
          case 1: 
            temp[i] = buff;
            i = 0;
            memcpy(&d[j], temp, sizeof(int16_t));
            j++; 
            if (j == (length)) state = 5; 
            break;
          }
        break;
      // Check for stop byte
      case 5: 
        if (buff == UART_STOP_BYTE) state = 6;  
        else return 0 ; 
        break; 
      // Get the checksum then check it. 
      case 6: 
        RXd_CS[0] = buff; state= 7; 
        break;
      case 7:
        RXd_CS[1] = buff; state= 8; 
        calculate_checksum(CALCd_CS, cs_buff, length*sizeof(uint16_t)+4);
        // Check the calculated and received checksum
        if ((CALCd_CS[0]==RXd_CS[0]) && (CALCd_CS[1]==RXd_CS[1]))  
          return 1;       // IF they match return 1
        else 
          return 0; 
        break;
      default: 
        state = 0; 
      }
    }  
   }
 } /*uart_recieve_uint16 */

/*************************************************************************************
 * Function:          uint8_t uart_recieve_int16(usci_a_uart_t *in_ptr, 
 *                                               uint32_t mclk_speed, int16_t * d, uint8_t ID)
 *
 * PreCondition:      1. start_device_uart has run
 *                    2. UART d has been sent and has been recieved by msp430
 *
 * Input:             Parameters:
 *                    - usci_a_uart_t *in_ptr: Uart A bus being used
 *                    - uint32_t mclk_speed: essentially a timeout for the wait uart busy fn.
 *                    - int16_t * d: Pointer to where the received data is going 
 *                    - uint8_t ID: Application specific identifier
 *  
 * Output:            1 if UART has correctly been received according to 'protocol' in UART.h
 *                    0 if something broke... 
 *
 * Side Effects:      Writes to int16_t * d 
 *  
 * Overview:          1. Waits for UART_Not_Busy to run with timeout
 *                    2. Cycles through ring buffer where recieved uart bytes are stored
 *                    3. Checks bytes and checksum, returns 0 if something didn't match
 *
 * Note:              None 
 *                  
 *************************************************************************************/
uint8_t uart_recieve_int16(usci_a_uart_t *in_ptr, uint32_t mclk_speed, int16_t * d, uint8_t ID) {
  
  // Variables 
  uint8_t buff, length, RXd_CS[2], CALCd_CS[2], cs_buff[512], temp[sizeof(int16_t)]; 
  uint8_t i=0;
  uint8_t j=0;
  uint16_t k = 0;
  volatile int state=0;

  // Wait for all UART transmissions to end
  if (Wait_for_UART_Not_Busy(in_ptr, mclk_speed)) 
    return 0; 
  // Once done start parsing d
  else
  {
    while(rbuffer_count(&UART_RX))
    {
     buff= rbuffer_getchar(&UART_RX);
     cs_buff[k] = buff;
     k++;
     switch(state) 
      {
      // Cases 0 - 2 are tell the type a
      case 0 :
        if (buff == UART_START_BYTE) state = 1;
        else return 0 ;
        break;
      case 1 : 
        if (buff == UART_INT16) state = 2; 
        else return 0;
        break;
      case 2 : 
        ID = buff; 
        state = 3;        
      case 3 : 
        length = buff; 
        state =4; 
        break; 
      // Get all the individual bytes to assemble into a d piece 
      case 4 : 
         switch(i) {
          case 0: 
            temp[i]= buff;
            i++;
            break;
          case 1: 
            temp[i] = buff;
            i = 0;
            memcpy(&d[j], temp, sizeof(int16_t));
            j++; 
            if (j == (length)) state = 5; 
            break;
          }
        break;
      // Check for stop byte
      case 5: 
        if (buff == UART_STOP_BYTE) state = 6;  
        else return 0 ; 
        break; 
      // Get the checksum then check it. 
      case 6: 
        RXd_CS[0] = buff; state= 7; 
        break;
      case 7:
        RXd_CS[1] = buff; state=8; 
        calculate_checksum(CALCd_CS, cs_buff, length*sizeof(int16_t)+4);
        // Check the calculated and received checksum
        if ((CALCd_CS[0]==RXd_CS[0]) && (CALCd_CS[1]==RXd_CS[1]))  
          return 1;       // IF they match return 1
        else 
          return 0; 
        break;
      default: 
        state = 0; 
      }
    }  
   }
 } /*uart_recieve_int16 */

 /*************************************************************************************
 * Function:          uint8_t uart_recieve_int8(usci_a_uart_t *in_ptr, 
 *                                               uint32_t mclk_speed, int8_t * d, uint8_t ID)
 *
 * PreCondition:      1. start_device_uart has run
 *                    2. UART d has been sent and has been recieved by msp430
 *
 * Input:             Parameters:
 *                    - usci_a_uart_t *in_ptr: Uart A bus being used
 *                    - uint32_t mclk_speed: essentially a timeout for the wait uart busy fn.
 *                    - int8_t * d: Pointer to where the received data is going 
 *                    - uint8_t ID: Application specific identifier
 *  
 * Output:            1 if UART has correctly been received according to 'protocol' in UART.h
 *                    0 if something broke... 
 *
 * Side Effects:      Writes to int8_t * d 
 *  
 * Overview:          1. Waits for UART_Not_Busy to run with timeout
 *                    2. Cycles through ring buffer where recieved uart bytes are stored
 *                    3. Checks bytes and checksum, returns 0 if something didn't match
 *
 * Note:              None 
 *                  
 *************************************************************************************/
uint8_t uart_recieve_int8(usci_a_uart_t *in_ptr, uint32_t mclk_speed, int8_t * d, uint8_t ID) {
  
  // Variables 
  uint8_t buff, length, RXd_CS[2], CALCd_CS[2], cs_buff[512], temp[sizeof(int16_t)]; 
  uint8_t i=0;
  uint8_t j=0;
  uint16_t k = 0;
  volatile int state=0;

  // Wait for all UART transmissions to end
  if (Wait_for_UART_Not_Busy(in_ptr, mclk_speed)) 
    return 0; 
  // Once done start parsing d
  else
  {
    while(rbuffer_count(&UART_RX))
    {
     buff= rbuffer_getchar(&UART_RX);
     cs_buff[k] = buff;
     k++;
     switch(state) 
      {
      // Cases 0 - 2 are tell the type a
      case 0 :
        if (buff == UART_START_BYTE) state = 1;
        else return 0 ;
        break;
      case 1 : 
        if (buff == UART_INT8) state = 2; 
        else return 0;
        break;
      case 2 : 
        ID = buff; 
        state = 3;      
      case 3 : 
        length = buff; 
        state =4; 
        break; 
      // Get all the individual bytes to assemble into a d piece 
      case 4 : 
        d[j] = (int8_t) buff;
        j++; 
        if (j == (length)) state = 5; 
        break;
      // Check for stop byte
      case 5: 
        if (buff == UART_STOP_BYTE) state = 6;  
        else return 0 ; 
        break; 
      // Get the checksum then check it. 
      case 6: 
        RXd_CS[0] = buff; state= 7; 
        break;
      case 7:
        RXd_CS[1] = buff; state=8; 
        calculate_checksum(CALCd_CS, cs_buff, length*sizeof(int8_t)+4);
        // Check the calculated and received checksum
        if ((CALCd_CS[0]==RXd_CS[0]) && (CALCd_CS[1]==RXd_CS[1]))  
          return 1;       // IF they match return 1
        else 
          return 0; 
        break;
      default: 
        state = 0; 
      }
    }  
   }
 } /*uart_recieve_int8 */

/*************************************************************************************
 * Function:          uint8_t uart_recieve_int8(usci_a_uart_t *in_ptr, 
 *                                               uint32_t mclk_speed, uint8_t * d, uint8_t ID)
 *
 * PreCondition:      1. start_device_uart has run
 *                    2. UART d has been sent and has been recieved by msp430
 *
 * Input:             Parameters:
 *                    - usci_a_uart_t *in_ptr: Uart A bus being used
 *                    - uint32_t mclk_speed: essentially a timeout for the wait uart busy fn.
 *                    - int8_t * d: Pointer to where the received data is going
 *                    - uint8_t ID: Application specific identifier
 *  
 * Output:            1 if UART has correctly been received according to 'protocol' in UART.h
 *                    0 if something broke... 
 *
 * Side Effects:      Writes to int8_t * d 
 *  
 * Overview:          1. Waits for UART_Not_Busy to run with timeout
 *                    2. Cycles through ring buffer where recieved uart bytes are stored
 *                    3. Checks bytes and checksum, returns 0 if something didn't match
 *
 * Note:              None 
 *                  
 *************************************************************************************/
uint8_t uart_recieve_uint8(usci_a_uart_t *in_ptr, uint32_t mclk_speed, uint8_t * d, uint8_t ID) {
  
  // Variables 
  uint8_t buff, length, RXd_CS[2], CALCd_CS[2], cs_buff[512], temp[sizeof(int16_t)]; 
  uint8_t i=0;
  uint8_t j=0;
  uint16_t k = 0;
  volatile int state=0;

  // Wait for all UART transmissions to end
  if (Wait_for_UART_Not_Busy(in_ptr, mclk_speed)) 
    return 0; 
  // Once done start parsing d
  else
  {
    while(rbuffer_count(&UART_RX))
    {
     buff= rbuffer_getchar(&UART_RX);
     cs_buff[k] = buff;
     k++;
     switch(state) 
      {
      // Cases 0 - 2 are tell the type a
      case 0 :
        if (buff == UART_START_BYTE) state = 1;
        else return 0 ;
        break;
      case 1 : 
        if (buff == UART_UINT8) state = 2; 
        else return 0;
        break;
      case 2 : 
        ID = buff; 
        state = 3;       
      case 3 : 
        length = buff; 
        state = 4; 
        break; 
      // Get all the individual bytes to assemble into a d piece 
      case 4 : 
        d[j] =buff;
        j++; 
        if (j == (length)) state = 5; 
        break;
      // Check for stop byte
      case 5: 
        if (buff == UART_STOP_BYTE) state = 6;  
        else return 0 ; 
        break; 
      // Get the checksum then check it. 
      case 6: 
        RXd_CS[0] = buff; state= 7; 
        break;
      case 7:
        RXd_CS[1] = buff; state=8; 
        calculate_checksum(CALCd_CS, cs_buff, length*sizeof(uint8_t)+4);
        // Check the calculated and received checksum
        if ((CALCd_CS[0]==RXd_CS[0]) && (CALCd_CS[1]==RXd_CS[1]))  
          return 1;       // IF they match return 1
        else 
          return 0; 
        break;
      default: 
        state = 0; 
      }
    }  
   }
 } /*uart_recieve_uint8 */

/********************************************************************************
*
* ISRs
*
 *******************************************************************************/
void USCI_A1_TX_ISR(void) __interrupt [USCIAB1TX_VECTOR]
{ 
 while (rbuffer_count(&UART_TX))
 {
 // Does not work without the delay in here.
   send_byte = rbuffer_getchar(&UART_TX);
   USCI_A_UART_transmitData(&UART_A1, send_byte);
   Wait_for_UART_Not_Busy( &UART_A1, MCLK_SPD);
 }
 USCI_A_UART_clearInterruptFlag(&UART_A1, UCA1TXIFG);
}


void USCI_A1_RX_ISR(void) __interrupt [USCIAB1RX_VECTOR] 
  {   
  // Get a byte, store a byte 
  got_byte = USCI_A_UART_receiveData(&UART_A1);
  rbuffer_putchar(&UART_RX, got_byte);
 }

