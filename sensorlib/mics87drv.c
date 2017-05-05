//*****************************************************************************
// MICS87drv.c - Accelerometer sensor driver APIs.
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "common.h"
#include "ads1115.h"
#include "mics87drv.h"
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
MICS87PowerUp()
{
	 g_MICS87Data.bActive = MICS87_POWERUP_MODE;
//First minute after Power-on
//Functional Test Mode
return SUCCESS;
}


int
MICS87PowerUpComplete()
{
	 g_MICS87Data.bActive = MICS87_CONTINUOS_MODE;
	 return SUCCESS;
}
//****************************************************************************
//
//! Initialize the MICS87 accelerometer device with defaults
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
MICS87Open()
{
    unsigned char ucRegVal;
    //
    // Read the CHIP ID NUM
    //
    RET_IF_ERR(GetADCRegisterValue(MICS87_CHID_ID_NUM, &ucRegVal));
    DBG_PRINT("CHIP ID: 0x%x\n\r", ucRegVal);

    return SUCCESS;
}

//****************************************************************************
//
//! Place the MICS87 accelerometer device to standby
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
MICS87Close()
{
    return SUCCESS;
}



float computeVOC(float analogValue)
{
	float CO2 = (analogValue) * (2000.0-400.0)/(5.0) + 400.0;

    return CO2;
}


//****************************************************************************
//
//!
//! \return 0: Success, < 0: Failure.
//
//********in********************************************************************

int MICS87Read( int* bActive, float* fVOC, char* logs)
{
	int logLen = 0;
	uint8_t writeBuf[3]={0};
	uint8_t readBuf[2]={0};
	uint8_t configPtr[1] = {1};//MICS87_CONFIG_REG;

	configPtr[0] = 1;

	  writeBuf[0] =	 1;// MICS87_CONFIG_REG;
	  writeBuf[1] =MICS87_CONFIG_0;
	  writeBuf[2] =MICS87_CONFIG_1;

	  RET_IF_ERR(I2C_IF_Write(MICS87_DEV_ADDR,writeBuf,3,1));

	  // read conversion register
		// write register pointer first
		  readBuf[0] = 0;   //set conversion register to 0 for reading
		  readBuf[1] = 0;   //bookkeeping

	  // wait for conversion complete
	  // checking bit 15
//	  long cnt = 0;
	  do {
		  RET_IF_ERR(I2C_IF_ReadFrom(MICS87_DEV_ADDR,configPtr, 1,
		                   readBuf, 2) );
		  //cnt++;
	  }
	  while (((readBuf[0] & 0x80) == 0) );//&& cnt < 3   0b 10000000


	  writeBuf[0] =	 0;// MICS87_CONFIG_REG;
	  configPtr[0] =	 0;// MICS87_CONFIG_REG;


	  RET_IF_ERR(I2C_IF_Write(MICS87_DEV_ADDR,writeBuf,1,1));


	  // read 2 bytes
	  RET_IF_ERR(I2C_IF_ReadFrom(MICS87_DEV_ADDR,configPtr, 1,
	                  readBuf, 2) );

	  // build 16bit integer
	  uint16_t  val = readBuf[0] << 8 | readBuf[1];

	  g_MICS87Data.raw = val;
		//normalize

	  float analog = (float)val*4.096/32768.0;

	  g_MICS87Data.normalized = analog;

	  float cVOC = computeVOC(analog);

		*bActive = SUCCESS;
		*fVOC =   cVOC;

		logs[logLen++]='v';
		logs[logLen++]='o';
		logs[logLen++]='c';
		logs[logLen++]='\0';


	  return SUCCESS;
}



//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
