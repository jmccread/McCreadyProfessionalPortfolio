/******************************************
 *  ____    ____   ____  ____   _____     *	
 * |_   \  /   _| |_  _||_  _| |_   _|    *
 *   |   \/   |     \ \  / /     | |      *
 *   | |\  /| |      > `' <      | |   _  *
 *  _| |_\/_| |_   _/ /'`\ \_   _| |__/ | *	
 * |_____||_____| |____||____| |________| *	
 *                                        *	
 ******************************************/

#include "stdint.h"
/* Code from online, wikipedia.com */
void fletcher16( uint8_t *checkA, uint8_t *checkB, uint8_t *data, uint16_t len )
{
        uint16_t sum1 = 0xff, sum2 = 0xff;
 
        while (len) {
                uint16_t tlen = len > 21 ? 21 : len;
                len -= tlen;
                do {
                        sum1 += *data++;
                        sum2 += sum1;
                } while (--tlen);
                sum1 = (sum1 & 0xff) + (sum1 >> 8);
                sum2 = (sum2 & 0xff) + (sum2 >> 8);
        }
        /* Second reduction step to reduce sums to 8 bits */
        sum1 = (sum1 & 0xff) + (sum1 >> 8);
        sum2 = (sum2 & 0xff) + (sum2 >> 8);
        *checkA = (uint8_t)sum1;
        *checkB = (uint8_t)sum2;
        return;
}

/*Modified code to compute a partial fletcher given an initial temporary checksum*/
uint16_t partialFletcher(uint16_t in_crc, uint8_t *data, uint16_t len){
		uint16_t return_crc;
		uint16_t sum1 = in_crc & 0xff;
		uint16_t sum2 = in_crc >> 8;
        while (len) {
                uint16_t tlen = len > 21 ? 21 : len;
                len -= tlen;
                do {
                        sum1 += *data++;
                        sum2 += sum1;
                } while (--tlen);
                sum1 = (sum1 & 0xff) + (sum1 >> 8);
                sum2 = (sum2 & 0xff) + (sum2 >> 8);
        }
        /* Second reduction step to reduce sums to 8 bits */
        sum1 = (sum1 & 0xff) + (sum1 >> 8);
        sum2 = (sum2 & 0xff) + (sum2 >> 8);
		return_crc = sum1 & 0xff;
		return_crc += (sum2 & 0xff) << 8;
        return return_crc;
}

void calculate_checksum( uint8_t *checksum, uint8_t *message, uint16_t length )
{
	fletcher16( checksum, checksum+1, message, length );
}


