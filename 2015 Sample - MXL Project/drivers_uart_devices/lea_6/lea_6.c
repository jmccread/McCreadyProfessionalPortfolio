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
* lea_6.c
******************************************************************************/
#include "lea_6.h"

/********************************************************************************
*
* Delcarations & Definitions 
*
 *******************************************************************************/

// Define messages to be sent to configure UBLOX 6 
uint8_t gps_rst[44] = {
          0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x68,
          0x24, 0x47, 0x50, 0x54, 0x58, 0x54, 0x2C, 0x30, 0x31, 0x2C, 0x30, 0x31,
          0x2C, 0x30, 0x32, 0x2C, 0x52, 0x65, 0x73, 0x65, 0x74, 0x74, 0x69, 0x6E, 
          0x67, 0x20, 0x47, 0x50, 0x53, 0x2A,  0x36, 0x38 } ;
uint8_t set_default[23] = {
          0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFB, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0xFF, 0xFF, 0x00, 0x10,  0x00, 0x00, 0x17, 0x2B, 0x7E}; 
uint8_t gga_off[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x24};
uint8_t gga_on[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x05, 0x38};
uint8_t gll_off[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
uint8_t gsa_off[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
uint8_t gsv_off[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
uint8_t gsv_on[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x01, 0x01,
    0x01, 0x01, 0x01, 0x01, 0x08, 0x4D};
uint8_t rmc_off[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x04, 0x40};
uint8_t vtg_off[16] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x01, 0x05, 0x47}; 
uint8_t nav_4g[44] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03,
    0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA,
    0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x2C};

// Define messages to be sent to poll for nmea strings 
const uint8_t poll_gga[15] = {0x24, 0x45, 0x49, 0x47, 0x50, 0x51, 0x2C, 0x47, 0x47, 
  0x41, 0x2A, 0x32, 0x37,  0x0D, 0x0A}; 

// Book keeping variables
uint8_t *msgs[num_msgs] = {gps_rst, set_default, gga_off, gll_off, gsa_off, gsv_off, rmc_off, vtg_off, 
	nav_4g};
/*uint8_t *msg_acks[num_msgs] = {gga_ack, gll_ack, gsa_ack, gsv_ack, rmc_ack, vtg_ack, 
	nav_4g_ack};*/
int ack_ack[num_msgs]; 
int check_ack[num_msgs]; 
int msg_lengths[num_msgs] = {44, 23, 16, 16, 16, 16, 16, 16, 44};

/********************************************************************************
*
* Function Definitions 
*
 *******************************************************************************/

/*************************************************************************************
 * Function:        void initialize_lea_6(void)       
 *
 * PreCondition:    Uart bus is released for use by MSP430 for configuring the 
 *									LEA_6 GPS module 
 *
 * Input:           None
 *
 * Output:          An error message if each message was not acknoweldged
 *
 * Side Effects:    Configures LEA_6 GPS module such that it only outputs sentances
 *                  when polled and so that it is in high altitude mode.
 *
 * Overview:      	
 *                  1. Initializes UART according to the settings in start_device_uart
 *                  2. Transmits configuration messages one at at time
 *                  3. Disables UART interrupts
 *
 * Note:            TO DO: 
 *                  1. Solve the issue of not getting all the bytes of the acknowledge 
 *                  and then perform a check on the acknowledge packer 
 *                  
 *************************************************************************************/
void initialize_lea_6(void){
  // Declare variables 
  int msgs_index = 0;
  int send_index = 0;
  int ack_index = 0; 
  uint8_t check =0; 

  rbuffer_clear(&UART_RX);
  rbuffer_clear(&UART_TX);

  // Configure UART, enable pins, enable interrupts
  USCI_A_UART_disableInterrupt (&UART_A1, USCI_A_UART_RECEIVE_INTERRUPT);
  USCI_A_UART_enableInterrupt (&UART_A1, USCI_A_UART_TRANSMIT_INTERRUPT);
  
  while (msgs_index<num_msgs) 
  {	
  // Load TX buffer
  rbuffer_putchars(&UART_TX, msgs[msgs_index], msg_lengths[msgs_index]);

  // Send a configuration message 
  USCI_A_UART_transmitData(&UART_A1, (rbuffer_getchar(&UART_TX)));
  
  // Wait until tx chars have been drained; 
  while(rbuffer_count(&UART_TX));
  msgs_index++;
  Wait_for_UART_Not_Busy( &UART_A1, MCLK_SPD);
  }

  // Disable interrupts 
  USCI_A_UART_enableInterrupt (&UART_A1, USCI_A_UART_RECEIVE_INTERRUPT);
  USCI_A_UART_disableInterrupt (&UART_A1, USCI_A_UART_TRANSMIT_INTERRUPT);
} 

/*************************************************************************************
 * Function:        uint8_t poll_gpgga(uint32_t timeout)
 *
 * PreCondition:    Uart bus is released for use by MSP430 for polling the 
 *                  LEA_6 GPS module 
 *
 * Input:           uint32_t timeout: number of processor cyecles to wait for
 * 									proper UART reception   
 *
 * Output:          uint8_t: TRUE or FALSE, true it didn't timeout, false if it did 
 *
 * Side Effects:    None
 *
 * Overview:      	
 *                  1. UART interrupts are enabled
 *                  2. Command is written to the GPS
 *                  3. Timeout is included to allow for 
 *
 * Note:            Lea6 only operates on 9600 baud 
 *                  
 *************************************************************************************/
uint8_t poll_gpgga() {
  uint8_t error; 
  uint16_t start;
  uint16_t t_left; 
  uint16_t wait_ms; 
  rbuffer_clear(&UART_RX);
  rbuffer_clear(&UART_TX);
  error = USCI_A_UART_queryStatusFlags(&UART_A1, USCI_A_UART_BUSY);
  // Enable interrupts 
  USCI_A_UART_clearInterruptFlag (&UART_A1, USCI_A_UART_RECEIVE_INTERRUPT);
  USCI_A_UART_enableInterrupt (&UART_A1, USCI_A_UART_RECEIVE_INTERRUPT);
  USCI_A_UART_enableInterrupt (&UART_A1, USCI_A_UART_TRANSMIT_INTERRUPT);
  
  // Clear whatever was in UART_RX
  rbuffer_clear(&UART_RX);
  rbuffer_clear(&UART_TX);

  // Load UART TX buffer
  rbuffer_putchars(&UART_TX, poll_gga, 15);

  // Start Transmission instruction to give GPGGA sentence 
  USCI_A_UART_transmitData(&UART_A1,  (rbuffer_getchar(&UART_TX))); 

  // Wait until all bytes have been TX'd . 
  while (rbuffer_count(&UART_TX)); 
 
  // Wait for a full sentence to be logged 
  start =  data.ms_counter;
  t_left = 6000 - start; 
  wait_ms = 30000;//3000; 

  // Wait for the data to returned for up to 3 seconds 
  if (wait_ms < t_left) 
    {
    while(data.ms_counter < (start+wait_ms)){
      if (check_sentance())
        return 1; 
      }
    }
  else if (wait_ms> t_left)
   {
    while (!(data.ms_counter == (wait_ms-t_left)))
    {
      if (check_sentance())
        return 1; 
    } 
   }

   // Disable interrupts 
  //USCI_A_UART_disableInterrupt (&UART_A1, USCI_A_UART_RECEIVE_INTERRUPT);
  //USCI_A_UART_disableInterrupt (&UART_A1, USCI_A_UART_TRANSMIT_INTERRUPT);

  // If not returned in that time return 0, this is a timeout 
  return 0;
  }

uint8_t check_sentance(void) { 
  if (((UART_RX.buff[UART_RX.inP-1] == '\n') && (UART_RX.buff[UART_RX.outP] == '$'))&&(UART_RX.buff[UART_RX.inP-2] == '\r')) {
    return 1; }
  else 
    return 0; 
	}
		
/*************************************************************************************
 * Function:          
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
int UBX_ACK( char *msg, char *RX_ACK)
{
  uint8_t ackPacket[10];
  int pos = 0;
  int i = 2;
  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	// header
  ackPacket[1] = 0x62;	// header
  ackPacket[2] = 0x05;	// class
  ackPacket[3] = 0x01;	// id
  ackPacket[4] = 0x02;	// length
  ackPacket[5] = 0x00;
  ackPacket[6] = msg[2];	// ACK class
  ackPacket[7] = msg[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums 
  for (i = 2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
  
   // Check that RX_ACK and ack_Packet match and return result 
  while(1) 
  {
   if (ackPacket[pos] == RX_ACK[pos]){
    pos++;
    if (pos>9) return 1;
    }
   else return 0; 
  }
}

/*************************************************************************************
 * Function:        	void preprocgps(void) 
 *
 * PreCondition:   	None
 *
 * Input:           	None	
 *
 * Output:           	None
 *
 * Side Effects:    	Writes 0 to everything in gpsmsg
 *
 * Overview:      			
 *	
 * Note:          	
 *                  
 *************************************************************************************/
void preprocgps(gps_struct *gpsmsg)	//Everything 0 or '\0'
{
	int i;

	gpsmsg->fix_status='\0';

	for(i=0;i<15;i++)
	{
		gpsmsg->lat_s[i]='\0';
		gpsmsg->lon_s[i]='\0';
		gpsmsg->alt_s[i]='\0';
		gpsmsg->time_s[i]='\0';
	}

	for(i=0;i<8;i++)
	{
		gpsmsg->lat_aprs[i]='\0';
		gpsmsg->lon_aprs[i]='\0';
	}
	gpsmsg->lon_aprs[i]='\0';

	for(i=0;i<6;i++)
	{
		gpsmsg->alt_aprs[i]='\0';
		gpsmsg->time_aprs[i]='\0';
	}
	gpsmsg->time_aprs[i]='\0';

	for(i=0;i<12;i++)
	{
		gpsmsg->lat_gsm[i]='\0';
		gpsmsg->lon_gsm[i]='\0';
	}

	gpsmsg->lat_f=0;
	gpsmsg->lon_f=0;
	gpsmsg->alt_f_ft=0;
	gpsmsg->alt_f_m=0;
}



/*************************************************************************************
 * Function:            uint8_t updateGPS(gps_struct *gpsmsg)       
 *
 * PreCondition:   	1. initialize_lea_6 has run and was successful
 *                      2. poll_gpgga has run and a full nmea sentance should be 
 *			in the buffer
 * 			3. preprocgps has run and initialized the state of the struct
 *
 * Input:           	gps_struct *gpsmsg: pointer global gps_struct
 *	
 * Output:           	Error based on switch case behavior 
 *
 * Side Effects:    	Updates Global gps_struct gpsmsg
 *
 * Overview:      		State machine that parses a recieved GPS sentances, works only 
 *										for GPGGA messages but could be adapted to include other types
 *
 * Note:          		Requires gpsmsg to be global container for parsed gps information
 * 										
 * 										TO DO: ????? 
 *                  
 *************************************************************************************/
uint8_t updateGPS(gps_struct *gpsmsg, char * nmea_str)
{
	char buff;
	volatile int state=0;
        int i = 0;
        int check_error=0;
	char *tme_ptr = gpsmsg->time_s;
	char *lat_ptr = gpsmsg->lat_s;
	char *lon_ptr = gpsmsg->lon_s;
	char *alt_ptr = gpsmsg->alt_s;  
        //char samp_sent[125] = "$GPGGA,134246.00,4208.94636,N,08414.03931,W,1,08,1.31,6382.8,M,-34.5,M,,*5A\n"; 
	
        while(rbuffer_count(&UART_RX))
        //while (samp_sent[i] != '\n')
	{
                buff= rbuffer_getchar( &UART_RX); 
                //buff = samp_sent[i]; 
                nmea_str[i] = buff; 
                i++;

		switch(state)
		{
		case 0:					//Starting State=0
			if(buff=='$')		//$ Detected
				state=1;		//Progress to State=1
			else
				state=0;
			break;

		case 1:					//State=1 $
			if(buff=='G')		//G Detected
				state=2;		//Progress to State=2
			else
				state=0;
			break;

		case 2:					//State=2 $G
			if(buff=='P')		//P Detected
				state=3;		//Progress to State=3
			else
				state=0;
			break;

		case 3:					//State=3 $GP
			if(buff=='G')		//G Detected
				state=4;		//Progress to State=4
			else
				state=0;
			break;

		case 4:					//State=4 $GPG
			if(buff=='G')		//G Detected
				state=5;		//Progress to State=5
			else
				state=0;
			break;

		case 5:					//State=5 $GPGG
			if(buff=='A')		//A Detected
				state=6;		//Progress to State=6
			else
				state=0;
			break;

		case 6:					//State=6 $GPGGA
			if(buff==',')		//, Detected
				state=7;		//Progress to State=7
			else
				state=0;
			break;

		case 7:					//State=7 $GPGGA,(time info here)
			if(buff==',')		//, Detected
				state=8;		//Progress to State=8
			else {
				*tme_ptr = buff; 
                                tme_ptr++; //Append byte to time container
                                }
			break;

		case 8:					//State=8 $GPGGA,time,(lat info here)
			if(buff==',')		//, Detected
				state=9;		//Progress to State=9
			else
				*lat_ptr++=buff;	//Append byte to lat container
			break;

		case 9:					//State=9 $GPGGA,time,lat,(N/S info here)
			if(buff==',')		//, Detected
				state=10;		//Progress to State=10
			else
				*lat_ptr=buff;	//Append byte to lat container
			break;

		case 10:				//State=10 $GPGGA,time,lat,N/S,(lon info here)
			if(buff==',')		//, Detected
				state=11;		//Progress to State=11
			else
				*lon_ptr++=buff;	//Append byte to lon container
			break;

		case 11:				//State=11 $GPGGA,time,lat,N/S,lon,(E/W info here)
			if(buff==',')		//, Detected
				state=12;		//Progress to State=12
			else
				*lon_ptr=buff;	//Append byte to lon container
			break;

		case 12:				//State=12 $GPGGA,time,lat,N/S,lon,E/W,(Fix status info here)
			if(buff==',')		//, Detected
				state=13;		//Progress to State=13
			else
				gpsmsg->fix_status=(buff-48);
			break;

		case 13:				//State=13 $GPGGA,time,lat,N/S,lon,E/W,FS,(No of Sat info here)
			if(buff==',')		//, Detected
				state=14;		//Progress to State=14
			break;

		case 14:				//State=14 $GPGGA,time,lat,N/S,lon,E/W,FS,NoS,(HDOP info here)
			if(buff==',')		//, Detected
				state=15;		//Progress to State=15
			break;

		case 15:				//State=15 $GPGGA,time,lat,N/S,lon,E/W,FS,NoS,HDOP,(Alt info here)
			if(buff==',')		//, Detected
				state=16;		//Progress to State=16
			else
				*alt_ptr++=buff;	//Append byte to alt container
			break;

		case 16:				//State=16 $GPGGA,time,lat,N/S,lon,E/W,FS,NoS,Alt,(Alt unit info here)
			if(buff==',')		//, Detected
				state=17;		//Progress to State=17
			break;

		case 17:				//State=17 $GPGGA,time,lat,N/S,lon,E/W,FS,NoS,Alt,M,(Altref/Geoid Sep info here)
			if(buff==',')		//, Detected
				state=18;		//Progress to State=18
			break;

		case 18:				//State=18 $GPGGA,time,lat,N/S,lon,E/W,FS,NoS,Alt,M,Altref,(Altref unit info here)
			if(buff==',')		//, Detected
				state=19;		//Progress to State=19
			break;

		case 19:				//State=19 $GPGGA,time,lat,N/S,lon,E/W,FS,NoS,Alt,M,Altref,M,(Diffage info here)
			if(buff==',')		//, Detected
				state=20;		//Progress to State=20
			break;
/*
		case 20:				//State=20 $GPGGA,time,lat,N/S,lon,E/W,FS,NoS,Alt,M,Altref,M,Diffage,(Diff station info here)
			if(buff==',')		//, Detected
				state=21;		//Progress to State=21
			break;
*/
		case 20:				//State=21 $GPGGA,time,lat,N/S,lon,E/W,FS,NoS,Alt,M,Altref,M,Diffage,Diffstation,(* here)
			if(buff=='*')		//* Detected
				state=21;		//Progress to State=22
			break;

		/*case 21:				//State=22 $GPGGA,time,lat,N/S,lon,E/W,FS,NoS,Alt,M,Altref,M,Diffage,Diffstation,*(chksum here)
			if(buff=='\n')		//\n Detected
				state=22;		//Progress to State=23

                                // Return no error 
                                return 0;
			break;

		default:
			state=0;
			break;*/
                case 21:				//State=22 $GPGGA,time,lat,N/S,lon,E/W,FS,NoS,Alt,M,Altref,M,Diffage,Diffstation,*(chksum here)
                  if(buff=='\n')		//\n Detected
                  {
			state=22;		//Progress to State=23
                       // nmea_str[i]='\0';
                        //data.nmea_length = i; 
                        //sprintf(data.nmea, "%s", nmea_str);
                       //return 0;
                       check_error=1;
                      
                       }
                  break;

		default:
			state=0;
			break;

		}
  }
  
  
  if (check_error==1)
  {
    // A full string was parsed
    return 0;
  }
  else
  {
    // A full string was not parsed. 
    return 0x01;
  }
}

/*************************************************************************************
 * Function:          void postprocgps(gps_struct *gpsmsg)	
 *
 * PreCondition:   		1. initialize_lea_6 has run and was successful
*				2. poll_gpgga has run and a full nmea sentance should be 
*				in the buffer
* 				3. preprocgps has run and initialized the state of the struct
*				4. updata_gps has parsed data and completed the last case of the 
*                               state machine
 *
 * Input:             None
 *
 * Output:            None
 *
 * Side Effects:      Updates values in global gps_struct gpsmsg
 *
 * Overview:          This function is magic, don't look inside. 
 *
 * Note:              Gross and disgusting code inside, run away.
 *										
 *                  
 *************************************************************************************/

void postprocgps(gps_struct *gpsmsg)			
//Interconvert between all formats. Very messy for now
{
	int i;
	float is_dec=0;

	char lat_tmp[10];
	char lon_tmp[11];
        char time_tmp[6]; 
	char buff;

	float lat_mm_f;
	float lon_mm_f;
	float temp;
        
	//Extract time and convert into APRS Format
	for(i=0;i<6;i++)
		gpsmsg->time_aprs[i]=gpsmsg->time_s[i];
	gpsmsg->time_aprs[6]='h';

        

	//Convert Lat chars to uint8's
        for(i=0;i<6;i++)
		time_tmp[i]=(gpsmsg->time_s[i]-48);
	for(i=0;i<4;i++)
		lat_tmp[i]=(gpsmsg->lat_s[i]-48);
	for(i=5;i<10;i++)
		lat_tmp[i]=(gpsmsg->lat_s[i]-48);
  
        // Convert uint8 array into uint8 for time
        gpsmsg->hour = (uint8_t)(time_tmp[0]*10 + time_tmp[1]);
        gpsmsg->min = (uint8_t)(time_tmp[2]*10 + time_tmp[3]);
        gpsmsg->sec = (uint8_t)(time_tmp[4]*10 + time_tmp[5]);
	//Convert Lat to float
	lat_mm_f = 10*lat_tmp[2]+lat_tmp[3]+0.1*lat_tmp[5]+0.01*lat_tmp[6];
        lat_mm_f = lat_mm_f+0.001*lat_tmp[7]+0.0001*lat_tmp[8]+0.00001*lat_tmp[9];
	lat_mm_f= lat_mm_f/60.0;
	gpsmsg->lat_f=10*lat_tmp[0]+lat_tmp[1]+lat_mm_f;

	//Round up Lat to two decimal places
	if(lat_tmp[7]>=5)
	{
		lat_tmp[6]++;
		if(lat_tmp[6]==10)
		{
			lat_tmp[6]=0;
			lat_tmp[5]++;
			if(lat_tmp[5]==10)
			{
				lat_tmp[5]=0;
				lat_tmp[3]++;
				if(lat_tmp[3]==10)
				{
					lat_tmp[3]=0;
					lat_tmp[2]++;
					if(lat_tmp[2]==6)
					{
						lat_tmp[2]=0;
						lat_tmp[1]++;
					}
				}
			}
		}
	}
      
	//Convert Lat uint8's to chars in APRS format
	for(i=0;i<4;i++)
		gpsmsg->lat_aprs[i]=(lat_tmp[i]+48);
	gpsmsg->lat_aprs[4]='.';
	for(i=5;i<7;i++)
		gpsmsg->lat_aprs[i]=(lat_tmp[i]+48);
	gpsmsg->lat_aprs[7]=gpsmsg->lat_s[10];

	//Convert Lon chars to uint8's
	for(i=0;i<5;i++)
		lon_tmp[i]=(gpsmsg->lon_s[i]-48);
	for(i=6;i<11;i++)
		lon_tmp[i]=(gpsmsg->lon_s[i]-48);

	//Convert Lon to float
        lon_mm_f = 10*lon_tmp[3]+lon_tmp[4]+0.1*lon_tmp[6]+0.01*lon_tmp[7];
	lon_mm_f = lon_mm_f + 0.001*lon_tmp[8]+0.0001*lon_tmp[9]+0.00001*lon_tmp[10];
	lon_mm_f= lon_mm_f/60.0;
	gpsmsg->lon_f=100*lon_tmp[0]+10*lon_tmp[1]+lon_tmp[2]+lon_mm_f;
      
	//Round up Lon to two decimal places
	if(lon_tmp[8]>=5)
	{
		lon_tmp[7]++;
		if(lon_tmp[7]==10)
		{
			lon_tmp[7]=0;
			lon_tmp[6]++;
			if(lon_tmp[6]==10)
			{
				lon_tmp[6]=0;
				lon_tmp[4]++;
				if(lon_tmp[4]==10)
				{
					lon_tmp[4]=0;
					lon_tmp[3]++;
					if(lon_tmp[3]==6)
					{
						lon_tmp[3]=0;
						lon_tmp[2]++;
					}
				}
			}
		}
	}



	//Convert Lon uint8's to chars in APRS format
	for(i=0;i<5;i++)
		gpsmsg->lon_aprs[i]=(lon_tmp[i]+48);
	gpsmsg->lon_aprs[5]='.';
	for(i=6;i<8;i++)
		gpsmsg->lon_aprs[i]=(lon_tmp[i]+48);
	gpsmsg->lon_aprs[8]=gpsmsg->lon_s[11];

	//Convert Alt(m) in chars to float
	for(i=0;i<15;i++)
	{
		buff=gpsmsg->alt_s[i];
		switch(buff)
		{
		case '.':
			is_dec=0.1;
			gpsmsg->alt_f_m*=0.1;
			break;

		case '\0':
			break;

		default:
			if(is_dec)
			{
				gpsmsg->alt_f_m +=is_dec*(buff-48);
				is_dec*=0.1;
			}
			else
			{

				gpsmsg->alt_f_m += (buff-48);
				gpsmsg->alt_f_m*=10;
			}
			break;
		}
	}
        
	//Convert Alt(m) to Alt(ft)
	gpsmsg->alt_f_ft = gpsmsg->alt_f_m*3.28084;
	//Convert Alt(ft) to chars in APRS, Note Precise to +/- 1 ft due to truncation of decimal point values
	for(i=5;i>=0;i--)
	{
		gpsmsg->alt_aprs[i] = (char) (((int)gpsmsg->alt_f_ft)%10 + 48);
		gpsmsg->alt_f_ft*= 0.1;

	}

	//Convert Lat to GSM rep
	temp = lat_mm_f*10000000.0;					//Still need to figure how floats work exactly

	for(i=11;i>4;i--)
	{
		gpsmsg->lat_gsm[i] = (char) (((long)temp)%10+48);
		temp*=0.1;
	}
	gpsmsg->lat_gsm[4]='.';
	temp=gpsmsg->lat_f;
	for(i=3;i>0;i--)
	{
		gpsmsg->lat_gsm[i] = (char) (((long)temp)%10+48);
		temp*=0.1;
	}
	if(gpsmsg->lat_aprs[7]=='N')
		gpsmsg->lat_gsm[0]='+';
	else
		gpsmsg->lat_gsm[0]='-';

	//Convert Lon to GSM rep
	temp=lon_mm_f*10000000.0;
	for(i=11;i>4;i--)
	{
		gpsmsg->lon_gsm[i] = (char) (((long)temp)%10+48);
		temp*=0.1;
	}
	temp=gpsmsg->lon_f;
	gpsmsg->lon_gsm[4]='.';
	for(i=3;i>0;i--)
	{
		gpsmsg->lon_gsm[i] = (char) (((long)temp)%10+48);
		temp*=0.1;
	}
	if(gpsmsg->lon_aprs[8]=='E')
		gpsmsg->lon_gsm[0]='+';
	else
		gpsmsg->lon_gsm[0]='-'; 
}