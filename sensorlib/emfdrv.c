//*****************************************************************************
// EMFdrv.c - Accelerometer sensor driver APIs.
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "common.h"
#include "ads1115.h"
#include "emfdrv.h"
// Common interface includes
#include "i2c_if.h"
#include "uart_if.h"


//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define FAILURE                 -1
#define SUCCESS                 0
#define RET_IF_ERR(Func)        {int iRetVal = (Func); \
                                 if (SUCCESS != iRetVal) \
                                     return  iRetVal;}
//#define DBG_PRINT               Report

//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                          
//****************************************************************************

int
EMFPowerUp()
{
	g_EMFData.bActive = EMF_POWERUP_MODE;
//First minute after Power-on
//Functional Test Mode
  return SUCCESS;
}


int
EMFPowerUpComplete()
{
	g_EMFData.bActive = EMF_CONTINUOS_MODE;
	return SUCCESS;
}
//****************************************************************************
//
//! Initialize the EMF accelerometer device with defaults
//!
//! \param None
//! 
//! This function  
//!    1. Reads the CHIP ID.
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
EMFOpen()
{
    unsigned char ucRegVal;
    //
    // Read the CHIP ID NUM
    //
    RET_IF_ERR(GetADCRegisterValue(EMF_CHID_ID_NUM, &ucRegVal));
    DBG_PRINT("CHIP ID: 0x%x\n\r", ucRegVal);

    return SUCCESS;
}

//****************************************************************************
//
//! Place the EMF accelerometer device to standby
//!
//! \param None
//! 
//! This function  
//!    1. Sets the device to standby mode.
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
EMFClose()
{
    return SUCCESS;
}

//50 mV/dB L
float computeSignalIntensity(float analogValue)
{
	float intensity = ((analogValue)/.051) - 52.0;

    return intensity;
}

//vout = vslope *(dbout - dbstart)
//dbout = vout/vslope + dbstart

//****************************************************************************
//
//! Get the emf data readings
//!
//! \param pfEmf pointer to the emf store
//! 
//! This function  
//!    1. Reads the data registers over I2C.
//!    2. Applies the range conversion to the raw values
//!
//! \return 0: Success, < 0: Failure.
//
//********in********************************************************************

int EMFRead( int* bActive, float* fEMF)
{

	uint8_t writeBuf[3];
	uint8_t readBuf[2];
	uint8_t configPtr[1];

	configPtr[0]= EMF_CONFIG_REG;

	  writeBuf[0] =	  EMF_CONFIG_REG;
	  writeBuf[1] =EMF_CONFIG_0;
	  writeBuf[2] =EMF_CONFIG_1;

	  RET_IF_ERR(I2C_IF_Write(EMF_DEV_ADDR,writeBuf,3,1));

	  readBuf[0] = 0;   // conversion register is 0
	  readBuf[1] = 0;

	  // wait for conversion complete
	  // checking bit 15
	  do {
		  RET_IF_ERR(I2C_IF_ReadFrom(EMF_DEV_ADDR,configPtr, 1,
		                  readBuf, 2) );
	  }
	  while (((readBuf[0] & 0x80) == 0));//&& cnt < MAX_CNT


		configPtr[0]= 0;


	  // read conversion register
	  // write register pointer first
	  readBuf[0] = 0;   // conversion register is 0
	  RET_IF_ERR(I2C_IF_Write(EMF_DEV_ADDR,readBuf,1,1));


	  // read 2 bytes
	  RET_IF_ERR(I2C_IF_ReadFrom(EMF_DEV_ADDR,configPtr, 1,
	                  readBuf, 2) );

	  // convert display results
	  uint16_t  val = readBuf[0] << 8 | readBuf[1];

	  g_EMFData.raw = val;

		//normalize
	  float analog = (float)val*4.096/32767.0;

	  g_EMFData.normalized = analog;


	  float fIntensity = computeSignalIntensity(analog);//watts per meter^2

	  if (fIntensity < 0.000000001 && fIntensity > -0.000000001)   fIntensity = 0.0;


	   *fEMF =fIntensity;
	   *bActive = SUCCESS;

	  return SUCCESS;
}





//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
