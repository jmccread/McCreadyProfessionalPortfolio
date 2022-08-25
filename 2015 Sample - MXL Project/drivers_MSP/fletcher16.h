/******************************************
 *  ____    ____   ____  ____   _____     *	
 * |_   \  /   _| |_  _||_  _| |_   _|    *
 *   |   \/   |     \ \  / /     | |      *
 *   | |\  /| |      > `' <      | |   _  *
 *  _| |_\/_| |_   _/ /'`\ \_   _| |__/ | *	
 * |_____||_____| |____||____| |________| *	
 *                                        *	
 ******************************************/

/******************************************************************************
* fletcher16.h
******************************************************************************/
#ifndef __fletcher16_h
#define __fletcher16_h

void fletcher16( uint8_t *checkA, uint8_t *checkB, uint8_t *data, uint16_t len );
uint16_t partialFletcher(uint16_t in_crc, uint8_t *data, uint16_t len); 
void calculate_checksum( uint8_t *checksum, uint8_t *message, uint16_t length ); 

#endif
