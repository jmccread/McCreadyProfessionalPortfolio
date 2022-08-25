/******************************************************************************


******************************************************************************/
// Includes
#include "config.h"
#include "bmp085.h" 
#include "i2c.h"
#include "salvo.h"
#include "task_bmp085.h" 
/*****************************************************************************
* Calibration value variables needed
*****************************************************************************/
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;
long b5; 

/*****************************************************************************
* Calibration Addresses to signal read
*****************************************************************************/
uint8_t calib_add[]= {0xAA, 0xAC, 0xAE,0xB0, 0xB2, 0XB4, 0xB6, 0XB8, 0xBA, 0xBC, 0xBE};

/*****************************************************************************
* Information variables
*****************************************************************************/

const float p0 = 101325;                        // Pressure at sea level (Pa)


/*************************************************************************************
 * Function:        void bmp085Calibration()
 *
 * PreCondition:    Task_bmp085 created and called from scheduler
 *
 * Input:           None.
 *
 * Output:          None
 *
 * Side Effects:    
 *
 * Overview:        Stores all of the bmp085's calibration values into global variables
 *                  Calibration values are required to calculate temp and pressure
 *                  This function should be called before the while loop in task_bmp085
 *
 * Note:            
 *                  
 *************************************************************************************/
void bmp085Calibration()
{
  // Just keep trucking... 
  ac1 = bmp085ReadInt(calib_add[0]);
  ac2 = bmp085ReadInt(calib_add[1]);
  ac3 = bmp085ReadInt(calib_add[2]);
  ac4 = (unsigned int) bmp085ReadInt(calib_add[3]);
  ac5 = (unsigned int) bmp085ReadInt(calib_add[4]);
  ac6 = (unsigned int) bmp085ReadInt(calib_add[5]);
  b1 = bmp085ReadInt(calib_add[6]);
  b2 = bmp085ReadInt(calib_add[7]);
  mb = bmp085ReadInt(calib_add[8]);
  mc = bmp085ReadInt(calib_add[9]);
  md = bmp085ReadInt(calib_add[10]);
  return;
}

/*************************************************************************************
 * Function:        bmp085ReadUT()
 *
 * PreCondition:    None, but used in conjunction with bmp085Calibration()
 *
 * Input:           None.
 *
 * Output:          Uncompensated temperature value ut
 *
 * Side Effects:    None
 *
 * Overview:        Write 3 bytes to EEPROM of the Bmp085 or 180 and then wait 4.5 ms
 *                  before sampling uncompensated temp.
 *
 * Note:            None
 *                  
 *************************************************************************************/
void bmp085ReadUT(void)
{
  // Function variables
  uint8_t ut_tx_data[] ={0xF4, 0x2E};
 
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  I2C_Write(&ut_tx_data, 0x02, BMP085_ADDR);

  // Wait at least 4.5ms, assuming OS_timer at 1 ms ticks in task
}

/*************************************************************************************
 * Function:        bmp085UP()
 *
 * PreCondition:    None, but used in conjunction with bmp085Calibration()
 *
 * Input:           None
 *
 * Output:          Uncompensated pressure value, up.
 *
 * Side Effects:    None
 *
 * Overview:        Read the uncompensated pressure value
 *
 * Note:            None
 *                  
 *************************************************************************************/
void bmp085ReadUP()
{ 
  uint8_t up_tx_data[] = {0xF4, 0x34};

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  I2C_Write(&up_tx_data, 0x02, BMP085_ADDR);
}

/*************************************************************************************
 * Function:        bmp085ReadUP() 
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          two byte signed integer 
 *
 * Side Effects:    None
 *
 * Overview:       used to read in data after writing to an address
 *                 inside the bmp085 or bmp180
 *
 * Note:            None
 *                  
 *************************************************************************************/
int bmp085ReadInt(uint8_t address)
{
  uint8_t data[2];

  I2C_Write(&address, 0x01, BMP085_ADDR);
  I2C_Read(&data, 0x02, BMP085_ADDR);

  return (int) data[0]<<8 | data[1];

}

/*************************************************************************************
 * Function:        short bmp085GetTemperature(unsigned int ut)
 *
 * PreCondition:    Task has ran bmp085calibration, ran bmp085readut()
 *
 * Input:           uncalibrated temperature (ut)
 *
 * Output:          two byte signed integer,  Value returned will be in units 
 *                  of 0.1 deg C 
 *
 * Side Effects:    None
 *
 * Overview:       Calculate temperature given ut.
 *
 * Note:            None
 *                  
 *************************************************************************************/

short bmp085GetTemperature(void)
{
  uint8_t ut_tx_data[] ={0xF6};
  long x1, x2;
  uint16_t ut;

  // Read two bytes from registers 0xF6 and 0xF7 after 4.5 ms delay in task 
  ut = (uint16_t)(bmp085ReadInt(ut_tx_data[0]));

  x1 = (((long)ut - (long) ac6) * (long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}

long bmp085GetPressure(void)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  unsigned long up = 0; 
  uint8_t up_tx_data[] ={0xF6};
  uint8_t data[2]; 
  uint8_t e; 
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  I2C_Write(&up_tx_data[0], 0x01, BMP085_ADDR);
  I2C_Read(&data, 2, BMP085_ADDR);


  
  up = (((unsigned long) data[0] << 16) | ((unsigned long) data[1] << 8) | (unsigned long) data[2]) >> (8-0);

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<0) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>0));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}