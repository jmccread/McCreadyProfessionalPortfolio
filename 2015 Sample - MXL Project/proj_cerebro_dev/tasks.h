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


/******************************************************************************
* Includes for all task .c files 
******************************************************************************/
#include "task_led.h"  
#include "task_measure_high_f.h"
#include "task_measure_low_f.h"
#include "task_develop.h"
#include "task_log.h"
#include "task_stamp_comm.h"
#include "task_check_error.h"
#include "task_heater_control.h"

/******************************************************************************
* OS Task Control Block Pointers 
******************************************************************************/
#define TASK_LED_P                  OSTCBP(1)
#define TASK_MEASURE_HIGH_F_P       OSTCBP(2)
#define TASK_MEASURE_LOW_F_P        OSTCBP(3)
#define TASK_STAMP_COMM_P           OSTCBP(4)
#define TASK_LOG_LOW_F_P            OSTCBP(5)
#define TASK_LOG_HIGH_F_P           OSTCBP(6)
#define TASK_CHECK_ERROR_P          OSTCBP(7)
#define TASK_DEVELOP_P              OSTCBP(8)
#define TASK_HEATER_CONTROL_P       OSTCBP(9)
#define TASK_SHADO_P                OSTCBP(10)

/******************************************************************************
* OS Delays needed for complex scheduling 
******************************************************************************/
#define TASK_LED_DELAY              100
#define TASK_MEASURE_HIGH_F_DELAY   100
#define TASK_MEASURE_LOW_F_DELAY    100
#define TASK_STAMP_COMM_DELAY       100
#define TASK_LOG_LOW_F_DELAY        100
#define TASK_LOG_HIGH_F_DELAY       100

 

