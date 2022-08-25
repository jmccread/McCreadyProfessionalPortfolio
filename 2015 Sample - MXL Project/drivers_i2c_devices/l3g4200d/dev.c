void measureL3G4200D( float * gyro_TEMP_output, float * gyro_output_X, float * gyro_output_Y, float * gyro_output_Z )
	{
 

	}

void configureL3G4200D( )
	{
/*************************************************************************************
* After a 5 ms powerup where the device reads its own registers follow this write 
* sequence to configure the device	
*		1. Write CTRL_REG2
*		2. Write CTRL_REG3
* 	3. Write CTRL_REG4
* 	4. Write CTRL_REG6
* 	5. Write Reference
* 	6. Write INT1_THS
* 	7. Write INT1_DUR
*		8. Write INT1_CFG
*		9. Write CTRL_REG5
*		10. Write CTRL_REG1
*************************************************************************************/
	
	/* 
	* CTRL_REG2:
	* configure high pass filter mode, and highpass cutoff at 0.1 Hz
	* assuming 100 Hz sample rate 
	*/
  setHighPassMode(L3G4200D_HPM_NORMAL);
	setHighPassFilterCutOffFrequencyLevel(L3G4200D_HPCF7)

	/* 
	* CTRL_REG3:
 	* Configure chip level interrupts, ignore this for now.
	*/

	/* 
	* CTRL_REG4:
 	* Configure to  
	*/
	}
