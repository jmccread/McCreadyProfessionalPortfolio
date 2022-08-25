/******************************************************************************
****                                                                       ****
**                                                                           **
task_log_data()
Log telemetry from voltage dividers, current sense resistors, 
BMP085 along with a time stamp counted in seconds from TB0 interrupt 
**                                                                           **
****                                                                       ****
******************************************************************************/

#include "main.h"
#include "config.h"
#include "events.h"
#include "init.h"
#include "task_log_data.h"
#include "csk_sd.h"
#include "thin_usr.h"

// Pumpkin Salvo headers
#include "salvo.h"
int strTmp_ind;
 

void task_log_data(void) {
  
  // Local variable declarations 
  F_FILE *file;
  unsigned long size;

  // init 
  csk_sd_open();
  f_initvolume();

  // Now we make the SD Card available to all tasks
  OSSignalBinSem(RSRC_SD_CARD_P);

  while (1) {
   
    // Must wait until SD Card resource is available.
    OS_WaitBinSem(RSRC_SD_CARD_P, OSNO_TIMEOUT);
    OS_WaitBinSem(LOG_DATA_P, OSNO_TIMEOUT); 
   
    __disable_interrupt(); 
    // Toggle chip select. 
    CS_SEL &= ~CS_SD;
    CS_DIR |=  CS_SD;
    CS_OUT &=  ~CS_SD;

    file = f_open("FTULOG.txt", "a+");

    // If successful, print data from data struct and 
    // append a new line to the file 
    if (file) {
      sprintf(data.data_str, "%lu, %f, %f, %l, %h\r\n", t_sec, data.v37, data.v74, 
      data.pressure, data.temp); 
      f_write(&data.data_str, 1, strlen(data.data_str), file);
      f_close(file);
      }
    __enable_interrupt();

    // Make the task in-eligible until the next time Log_DATA_P is signaled 
    OSSignalBinSem(RSRC_SD_CARD_P); 
    //OSSignalBinSem(GET_TIME_P); 
    // Leave LOG_DATA_P unsignaled so that log occurs at regular time intervals 
    // Controlled by the timer B interrupt. 
    OS_Delay(10); 
    __enable_interrupt();

  } /* while */
  
} /* data_log.c */
