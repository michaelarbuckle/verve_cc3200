//*****************************************************************************
// BMA222drv.c - Accelerometer sensor driver APIs.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup oob
//! @{
//
//*****************************************************************************
#include <stdio.h>
#include <math.h>
#include "bma222drv.h"
// Common interface includes
#include "common.h"
#include "i2c_if.h"
#include "uart_if.h"
#include "osi.h"


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


//****************************************************************************
//
//! Returns the value in the specified register
//!
//! \param ucRegAddr is the offset register address
//! \param pucRegValue is the pointer to the register value store
//! 
//! This function  
//!    1. Returns the value in the specified register
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
GetRegisterValue(unsigned char ucRegAddr, unsigned char *pucRegValue)
{
    //
    // Invoke the readfrom  API to get the required byte
    //
    if(I2C_IF_ReadFrom(BMA222_DEV_ADDR, &ucRegAddr, 1,
                   pucRegValue, 1) != 0)
    {
        DBG_PRINT("I2C readfrom failed\n\r");
        return FAILURE;
    }

    return SUCCESS;
}

//****************************************************************************
//
//! Sets the value in the specified register
//!
//! \param ucRegAddr is the offset register address
//! \param ucRegValue is the register value to be set
//! 
//! This function  
//!    1. Returns the value in the specified register
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
SetRegisterValue(unsigned char ucRegAddr, unsigned char ucRegValue)
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
    if(I2C_IF_Write(BMA222_DEV_ADDR,ucData,2,1) == 0)
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
BlockRead(unsigned char ucRegAddr, 
          unsigned char *pucBlkData,
          unsigned char ucBlkDataSz)
{
    //
    // Invoke the readfrom I2C API to get the required bytes
    //
    if(I2C_IF_ReadFrom(BMA222_DEV_ADDR, &ucRegAddr, 1,
                   pucBlkData, ucBlkDataSz) != 0)
    {        
        DBG_PRINT("I2C readfrom failed\n");
        return FAILURE;
    }

    return SUCCESS;
}

//****************************************************************************
//
//! Initialize the BMA222 accelerometer device with defaults
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
BMA222Open()
{
    unsigned char ucRegVal;
    //
    // Read the CHIP ID NUM
    //
    RET_IF_ERR(GetRegisterValue(BMA222_CHID_ID_NUM, &ucRegVal));
    DBG_PRINT("CHIP ID: 0x%x\n\r", ucRegVal);

    return SUCCESS;
}

//****************************************************************************
//
//! Place the BMA222 accelerometer device to standby
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
BMA222Close()
{
    return SUCCESS;
}

//****************************************************************************
//
//! Get the accelerometer data readings
//!
//! \param pfAccX pointer to the AccX store
//! \param pfAccY pointer to the AccY store
//! \param pfAccZ pointer to the AccZ store
//! 
//! This function  
//!    1. Reads the data registers over I2C.
//!    2. Applies the range conversion to the raw values
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
BMA222Read(signed char *pcAccX, signed char *pcAccY, signed char *pcAccZ)
{
    char cAccX = 0;
    char cAccY = 0;
    char cAccZ = 0;
    //
    // Read the acclerometer output registers LSB and MSB
    //
    RET_IF_ERR(BlockRead(BMA222_ACC_DATA_X, (unsigned char *)&cAccX,
                     sizeof(cAccX)));

    RET_IF_ERR(BlockRead(BMA222_ACC_DATA_Y, (unsigned char *)&cAccY,
             sizeof(cAccY)));

    RET_IF_ERR(BlockRead(BMA222_ACC_DATA_Z, (unsigned char *)&cAccZ,
             sizeof(cAccZ)));

    *pcAccX = cAccX;
    *pcAccY = cAccY;
    *pcAccZ = cAccZ;

    return SUCCESS;
}

//****************************************************************************
//
//! Get the raw accelerometer data register readings
//!
//! \param psAccX pointer to the raw AccX store
//! \param psAccY pointer to the raw AccY store
//! \param psAccZ pointer to the raw AccZ store
//! 
//! This function  
//!    1. Reads the data registers over I2C.
//!    2. Returns the accelerometer readings
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int
BMA222ReadNew(signed char *pcAccX, signed char *pcAccY, signed char *pcAccZ)
{
    char cAccX[6];
   
    //
    // Read the acclerometer output registers LSB and MSB
    //
    RET_IF_ERR(BlockRead(BMA222_ACC_DATA_X_NEW, (unsigned char *)cAccX,6));

     //
    // Check whether new Sensor Data is available
    //
    if((cAccX[0] & 0x1) && (cAccX[2] & 0x1) && (cAccX[4] & 0x1))
    {
        *pcAccX = cAccX[1];
        *pcAccY = cAccX[3];
        *pcAccZ = cAccX[5];
        return SUCCESS;
    }

    //New Sensor Data Not Available
    return FAILURE;

}

//*****************************************************************************
//
//! ReadAccSensor
//!
//!    @brief  Read Accelerometer Data from Sensor
//!
//!
//!     @return none
//!
//!
//
//*****************************************************************************
int ReadAccSensor()
{
    //Define Accelerometer Threshold to Detect Movement
    const short csAccThreshold    = 5;

    signed char cAccXT1,cAccYT1,cAccZT1;
    signed char cAccXT2,cAccYT2,cAccZT2;
    signed short sDelAccX, sDelAccY, sDelAccZ;
    int iRet = -1;
    int iCount = 0;
    int motion = 0;

    iRet = BMA222ReadNew(&cAccXT1, &cAccYT1, &cAccZT1);
    if(iRet)
    {
        //In case of error/ No New Data return
        return -1;
    }
    for(iCount=0;iCount<2;iCount++)
    {
    	osi_Sleep(300); //30msec
        iRet = BMA222ReadNew(&cAccXT2, &cAccYT2, &cAccZT2);
        if(iRet)
        {
            //In case of error/ No New Data continue
            iRet = 0;
            continue;
        }

        else
        {
            sDelAccX = abs((signed short)cAccXT2 - (signed short)cAccXT1);
            sDelAccY = abs((signed short)cAccYT2 - (signed short)cAccYT1);
            sDelAccZ = abs((signed short)cAccZT2 - (signed short)cAccZT1);

            //Compare with Pre defined Threshold
            if(sDelAccX > csAccThreshold || sDelAccY > csAccThreshold ||
               sDelAccZ > csAccThreshold)
            {
                //Device Movement Detected, Break and Return
                motion = 1;
                break;
            }
            else
            {
                //Device Movement Static
                motion = 0;
            }
        }
    }
    return motion;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
