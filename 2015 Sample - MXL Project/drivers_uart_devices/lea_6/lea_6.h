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
* lea_6.h
******************************************************************************/
#ifndef __lea_6_h
#define __lea_6_h

/********************************************************************************
*
* Includes
*
 *******************************************************************************/
#include "stdint.h"
#include "uart.h"
#include "main.h"
#include "driverlib.h" 
#include "usci_a_uart.h"

/********************************************************************************
*
* Definitions & Macros 
*
 *******************************************************************************/
#define num_msgs              9			// Number of config msgs sent before polling
#define BAUD_RATE             9600
/********************************************************************************
*
* Declarations 
*
 *******************************************************************************/
typedef struct{

	char fix_status;

	char lat_s[15];					//NMEA Lat buffer
	char lon_s[15];					//NMEA Lon buffer
	char alt_s[15];					//NMEA Alt buffer (in m)
	char time_s[15];				//NMEA Time buffer

	char lat_aprs[8];				//APRS representation of Lat
	char lon_aprs[9];				//APRS representation of Lon
	char alt_aprs[6];				//APRS representation of Alt (in ft)
	char time_aprs[7];				//APRS representation of Time

	char lat_gsm[12];				//GSM representation of Time
	char lon_gsm[12];				//GSM representation of Time

	float lat_f;					//Float value of Lat
	float lon_f;					//Float value of Lon
	float alt_f_ft;					//Float value of Alt (in ft)
	float alt_f_m;					//Float value of ALt (in m)
        
        uint8_t hour; 
        uint8_t min;
        uint8_t sec; 

}  gps_struct;  

//extern gps_struct gpsmsg; 
/********************************************************************************
*
* Function Prototypes 
*
 *******************************************************************************/
void initialize_lea_6(void);
uint8_t poll_gpgga(void); 
int UBX_ACK( char *msg, char *RX_ACK);
void preprocgps(gps_struct *gpsmsg);
void postprocgps(gps_struct *gpsmsg);
uint8_t updateGPS(gps_struct * gpsmsg, char * nmea_str); 
#endif /*_task_lea_6_h
