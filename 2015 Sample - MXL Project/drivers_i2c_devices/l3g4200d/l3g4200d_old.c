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
 * date:	07/03/2014
 */
//#include "task_l3g4200d.h"
#include "l3g4200d.h"
#include "i2c.h"
#include "msp430.h"
#include "stdint.h"
#include "common_i2c_ops.h"



/*************************************************************************************
 * Function:     		          
 *
 * PreCondition:    
 *
 * Input:           Parameters:
 *                    - 
 *                    - 
 *                    - 
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
void measureL3G4200D(   float * gyro_TEMP_output, 
                    float * gyro_output_X, float * gyro_output_Y, float * gyro_output_Z )
{
  uint8_t dev_addr = l3g4200d_1SAD_ADDR; 
  int16_t gyro_TEMP_output_temp; 
  int16_t gyro_output_temp_X; 
  int16_t gyro_output_temp_Y;
  int16_t gyro_output_temp_Z; 
  uint8_t status_reg_out; 

/********************************************************************************
*
* Collect 2 byte data from measurement registers and form into an integer for 
* later conversion to floating point degree per second measurement 
*
*******************************************************************************/
  
  uint8_t w_reg4[] = {L3G4200D_RA_CTRL_REG4, 0xA0};
  uint8_t w_reg5[] = {L3G4200D_RA_CTRL_REG5, 0x40};
  uint8_t w_reg1[] = {L3G4200D_RA_CTRL_REG1, 0x0F};
  uint8_t start_read = 0xA6; 
  uint8_t Read_Byte_Array[10]; 
  I2C_Write(&w_reg4, 0x02, dev_addr);
  I2C_Write(&w_reg5, 0x02, dev_addr);
  I2C_Write(&w_reg1, 0x02, dev_addr);
  
  I2C_Write(&start_read, 0x01, dev_addr); 

  I2C_Read(&Read_Byte_Array ,0x07, dev_addr);


  // Processing of Data
  gyro_TEMP_output_temp = (int16_t)(Read_Byte_Array[0]);
  status_reg_out = (Read_Byte_Array[1]);
  gyro_output_temp_X = ((int16_t)Read_Byte_Array[3] << 8 | (int16_t)Read_Byte_Array[2]);
  gyro_output_temp_Y = ((int16_t)Read_Byte_Array[5] << 8 | (int16_t)Read_Byte_Array[4]);
  gyro_output_temp_Z = ((int16_t)Read_Byte_Array[7] << 8 | (int16_t)Read_Byte_Array[6]);


  *gyro_TEMP_output = (float)(gyro_TEMP_output_temp);
  *gyro_output_X = (float)gyro_output_temp_X;
  *gyro_output_Y = (float)gyro_output_temp_Y;
  *gyro_output_Z = (float)gyro_output_temp_Z;

 
}
