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
* Main.h 
************************************************************/
#ifndef __main_h
#define __main_h

/************************************************************ 
* Includes
************************************************************/
//Tasks.h contains all .h files and control block pointers for tasks 
#include "tasks.h"

// Other includes 
#include "usci_a_uart.h"
#include "telem_points.h"
#include "init.h"
#include <__cross_studio_io.h>
#include "config.h"
#include "error.h"
#include "stdint.h"
#include <math.h>
#include "effs_thin_mmc_drv_ucb0.h"
#include "thin_usr.h"
#include <string.h>

// Bus Includes 
#include "bus_isolation.h"
#include "usci_a_uart.h"
#include "uart.h"
#include "driverlib.h" 
#include "i2c.h"
#include "i2cdev.h"
#include "common_i2c_ops.h"

//Salvo 
#include "events.h"
#include "salvo.h"

/************************************************************ 
* Globals 
************************************************************/ 
extern data_struct data; 
extern error_struct e; 
extern data_struct check; 

extern uint8_t check_e(); 
extern uint8_t check_sd_e(); 
float update_time(uint8_t * H, uint8_t * M, float * S, uint8_t * set_time); 
#endif /* __main_h */
