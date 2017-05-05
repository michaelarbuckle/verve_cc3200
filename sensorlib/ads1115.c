#include "common.h"
#include "ads1115.h"
#include "i2c_if.h"

#define FAILURE                 -1
#define SUCCESS                 0
#define RET_IF_ERR(Func)        {int iRetVal = (Func); \
                                 if (SUCCESS != iRetVal) \
                                     return  iRetVal;}
//#define DBG_PRINT               Report
int i2CToADCInit()
{
//
// I2C Init
//
	int lRetVal = I2C_IF_Open(I2C_MASTER_MODE_FST);

return lRetVal;
}


int
SetADCRegisterValue(unsigned char ucRegAddr, unsigned char ucRegValue)
{
    unsigned char ucData[2];
    //
    // Select the register to be written followed by the value.
    //
    ucData[0] = ucRegAddr;
    ucData[1] = ucRegValue;
    //
    // Initiate the I2C write
    //
    if(I2C_IF_Write(ADC_DEV_ADDR,ucData,2,1) == 0)
    {
        return SUCCESS;
    }
    else
    {
        DBG_PRINT("I2C write failed\n\r");
    }

    return FAILURE;
}

//****************************************************************************
//
//! Reads a block of continuous data
//!
//! \param ucRegAddr is the start offset register address
//! \param pucBlkData is the pointer to the data value store
//! \param ucBlkDataSz is the size of data to be read
//! 
//! This function  
//!    1. Returns the data values in the specified store
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
ADCBlockRead(unsigned char ucRegAddr, 
          unsigned char *pucBlkData,
          unsigned char ucBlkDataSz)
{
    //
    // Invoke the readfrom I2C API to get the required bytes
    //
    if(I2C_IF_ReadFrom(ADC_DEV_ADDR, &ucRegAddr, 1,
                   pucBlkData, ucBlkDataSz) != 0)
    {        
        DBG_PRINT("I2C readfrom failed\n");
        return FAILURE;
    }

    return SUCCESS;
}


int
GetADCRegisterValue(unsigned char ucRegAddr, unsigned char *pucRegValue)
{
    //
    // Invoke the readfrom  API to get the required byte
    //
    if(I2C_IF_ReadFrom(ADC_DEV_ADDR, &ucRegAddr, 1,
                   pucRegValue, 1) != 0)
    {
        DBG_PRINT("I2C readfrom failed\n\r");
        return FAILURE;
    }

    return SUCCESS;
}
