//*****************************************************************************
// BME280drv.c - sensor driver APIs.
#include <stdio.h>
#include "stdint.h"
#include <math.h>
#include "common.h"
#include "utils.h"
#include "bme280drv.h"

// Common interface includes
#include "i2c_if.h"
#include "uart_if.h"

//****************************************************************************//
//
//  Settings and configuration
//
//****************************************************************************//


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
GetRegisterValueBME280(unsigned char ucRegAddr, unsigned char *pucRegValue)
{
    //
    // Invoke the readfrom  API to get the required byte
    //
    if(I2C_IF_ReadFrom(BME280_DEV_ADDR, &ucRegAddr, 1,
                   pucRegValue, 1) != 0)
    {
        DBG_PRINT("I2C readfrom failed\n\r");
        return FAILURE;
    }

    return SUCCESS;
}
uint8_t readRegister(uint8_t offsett)
{
	unsigned char  pucRegValue;
	if (I2C_IF_ReadFrom(BME280_DEV_ADDR, &offsett, 1,&pucRegValue, 1) != 0)
	{
        DBG_PRINT("I2C readfrom failed\n\r");
		return 0;
	}
    return pucRegValue;

}

int
GetRegisterValueShort(unsigned char ucRegAddr, unsigned short *pusRegValue)
{
    unsigned char ucRegData[2];
    //
    // Invoke the readfrom I2C API to get the required byte
    //
    if(I2C_IF_ReadFrom(BME280_DEV_ADDR, &ucRegAddr, 1,
                   &ucRegData[0], 2) != 0)
    {
        DBG_PRINT("I2C readfrom failed\n\r");
        return FAILURE;
    }

    *pusRegValue = (unsigned short)(ucRegData[0] << 8) | ucRegData[1];

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
SetRegisterValueBME280(unsigned char ucRegAddr, unsigned char ucRegValue)
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
    //
    /* int I2C_IF_Write(unsigned char ucDevAddr,
             unsigned char *pucData,
             unsigned char ucLen,
             unsigned char ucStop)
    */
    if(I2C_IF_Write(BME280_DEV_ADDR,ucData,2,1) == 0)
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
BlockReadBME280(unsigned char ucRegAddr,
          unsigned char *pucBlkData,
          unsigned char ucBlkDataSz)
{
    //
    // Invoke the readfrom I2C API to get the required bytes
    //
    if(I2C_IF_ReadFrom(BME280_DEV_ADDR, &ucRegAddr, 1,
                   pucBlkData, ucBlkDataSz) != 0)
    {        
        DBG_PRINT("I2C readfrom failed\n");
        return FAILURE;
    }

    return SUCCESS;
}




int BME280ReadTestValues( int* bActive, float* fHumidity,float* fPressure, float* fAltitude, float* fTemperature)
{

	//g_BME280Data.fTemperature =	11.1;
			g_BME280Data.fTemperatureF =	56.0;
			g_BME280Data.fPressure =	11001;
			g_BME280Data.fAltitude =	3;
			g_BME280Data.fAltitudeF =	9;
			g_BME280Data.fHumidity =	67.0;

	*bActive = g_BME280Data.bActive;
	*fHumidity =  g_BME280Data.fHumidity;
	*fPressure =  g_BME280Data.fPressure;
	*fAltitude = g_BME280Data.fAltitude;
	*fTemperature = g_BME280Data.fTemperature;
return SUCCESS;
}
/*
displays the BME280's physical memory and what the driver perceives the calibration words to be.

*/
void logBME280Registers(unsigned char* logs)
{

	unsigned char registers=0xFE;
	unsigned char dt[8];
	unsigned char cnt = 8;

	int retval;
	retval = BlockReadBME280(registers,dt,cnt);
	int z;
	for (z=0;z < cnt;z++ )
	{
		logs[z]=dt[z];
	}
	logs[z]='\0';
	//0xF7 to 0xFE

	}
/*
hum_lsb 0xFE 0x00
hum_msb 0xFD 0x80x
temp_xlsb 0xFC 0 0 0 0 0x00
temp_lsb 0xFB 0x00
temp_msb 0xFA 0x80
press_xlsb 0xF9 0 0 0 0 0x00
press_lsb 0xF8 0x00
press_msb 0xF7 0x80
config 0xF5 spi3w_en[0] 0x00
ctrl_meas 0xF4 0x00
status 0xF3 measuring[0] im_update[0] 0x00
ctrl_hum 0xF2
*/









void BME280run()
{
	//***Driver settings********************************//
	//commInterface can be I2C_MODE or SPI_MODE
	//specify chipSelectPin using arduino pin names
	//specify I2C address.  Can be 0x77(default) or 0x76

	settings.I2CAddress = 0x77;


	settings.commInterface = 0;
	settings.chipSelectPin = 0;

	//***Operation settings*****************************//

	//renMode can be:
	//  0, Sleep mode
	//  1 or 2, Forced mode
	//  3, Normal mode
	settings.runMode = 3; //Normal mode

	//tStandby can be:
	//  0, 0.5ms
	//  1, 62.5ms
	//  2, 125ms
	//  3, 250ms
	//  4, 500ms
	//  5, 1000ms
	//  6, 10ms
	//  7, 20ms
	settings.tStandby = 0;

	//filter can be off or number of FIR coefficients to use:
	//  0, filter off
	//  1, coefficients = 2
	//  2, coefficients = 4
	//  3, coefficients = 8
	//  4, coefficients = 16
	settings.filter = 0;

	//tempOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	settings.tempOverSample = 1;

	//pressOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
    settings.pressOverSample = 1;

	//humidOverSample can be:
	//  0, skipped
	//  1 through 5, oversampling *1, *2, *4, *8, *16 respectively
	settings.humidOverSample = 1;

	//Serial.begin(57600);//what is speed mode ??
    //I2C_INIT ????


	//Calling .begin() causes the settings to be loaded
	UtilsDelay(10);//	delay(10);  //Make sure sensor had enough time to turn on. BME280 requires 2ms to start up.

	//write request to Sensor
	uint8_t beginVal = BME280Begin();

	if (beginVal != BME280_CHIP_ID)
	{
		//Not reading sensor registers
	}

    //look at values
	uint8_t initRegisterData[4];

	initRegisterData[0]=readRegister(BME280_CHIP_ID_REG);
	initRegisterData[1]=readRegister(BME280_RST_REG);
	initRegisterData[2]=readRegister(BME280_CTRL_MEAS_REG);
	initRegisterData[3]=readRegister(BME280_CTRL_HUMIDITY_REG);


}


void updateBME280Data()
{
	//Each loop, take a reading.
	//Start with temperature, as that data is needed for accurate compensation.
	//Reading the temperature updates the compensators of the other functions
	//in the background.

	g_BME280Data.fTemperature =	readTempC();
	g_BME280Data.fTemperatureF =		readTempF();
	g_BME280Data.fPressure =	readFloatPressure();
	g_BME280Data.fAltitude =	readFloatAltitudeMeters();
	g_BME280Data.fAltitudeF =	readFloatAltitudeFeet();
	g_BME280Data.fHumidity =	readFloatHumidity();

	UtilsDelay(1000);
	//delay(1000);

}


//****************************************************************************
//
//! Initialize the BME280 accelerometer device with defaults
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
BME280Open()
{
    unsigned char ucRegVal;
    //
    // Read the CHIP ID NUM
    //
    RET_IF_ERR(
    		GetRegisterValueBME280(BME280_CHIP_ID_REG, &ucRegVal));
    DBG_PRINT("CHIP ID:  0x%x\n\r", ucRegVal);

    if(ucRegVal != BME280_CHIP_ID)
     {
         DBG_PRINT("Error in chip ID\n\r");
         return FAILURE;
     }


    return SUCCESS;
}

//****************************************************************************
//
//! Place the BME280 accelerometer device to standby
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
BME280Close()
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

int BME280Read( int* bActive, float* fHumidity,float* fPressure, float* fAltitude, float* fTemperature)
{
 /*   float lPressure = 0.0f;
    float lAltitude = 0.0f;
    float lTemperature = 0.0f;
*/
	    BME280run();

	    *fTemperature =	readTempC();
	    *fPressure =	readFloatPressure();
	    *fAltitude =	readFloatAltitudeMeters();
	    *fHumidity =	readFloatHumidity();

		*bActive = SUCCESS;

	    /*
	    g_BME280Data.fTemperature =	readTempC();
		g_BME280Data.fTemperatureF =		readTempF();
		g_BME280Data.fPressure =	readFloatPressure();
		g_BME280Data.fAltitude =	readFloatAltitudeMeters();
		g_BME280Data.fAltitudeF =	readFloatAltitudeFeet();
		g_BME280Data.fHumidity =	readFloatHumidity();




		*bActive = g_BME280Data.bActive;
		*fHumidity =  g_BME280Data.fHumidity;
		*fPressure =  g_BME280Data.fPressure;
		*fAltitude = g_BME280Data.fAltitude;
		*fTemperature = g_BME280Data.fTemperature;
*/
    return SUCCESS;
}


//Constructor -- Specifies default configuration


//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "mySensor.settings.commInterface = SPI_MODE;" to
//  configure before calling .begin();
//
//****************************************************************************//
uint8_t BME280Begin()
{
	//Check the settings structure values to determine how to setup the device
	uint8_t dataToWrite = 0;  //Temporary variable

//	Wire.begin();

	//Reading all compensation data, range 0x88:A1, 0xE1:E7

	calibration.dig_T1 = ((uint16_t)((readRegister(BME280_DIG_T1_MSB_REG) << 8) + readRegister(BME280_DIG_T1_LSB_REG)));
	calibration.dig_T2 = ((int16_t)((readRegister(BME280_DIG_T2_MSB_REG) << 8) + readRegister(BME280_DIG_T2_LSB_REG)));
	calibration.dig_T3 = ((int16_t)((readRegister(BME280_DIG_T3_MSB_REG) << 8) + readRegister(BME280_DIG_T3_LSB_REG)));

	calibration.dig_P1 = ((uint16_t)((readRegister(BME280_DIG_P1_MSB_REG) << 8) + readRegister(BME280_DIG_P1_LSB_REG)));
	calibration.dig_P2 = ((int16_t)((readRegister(BME280_DIG_P2_MSB_REG) << 8) + readRegister(BME280_DIG_P2_LSB_REG)));
	calibration.dig_P3 = ((int16_t)((readRegister(BME280_DIG_P3_MSB_REG) << 8) + readRegister(BME280_DIG_P3_LSB_REG)));
	calibration.dig_P4 = ((int16_t)((readRegister(BME280_DIG_P4_MSB_REG) << 8) + readRegister(BME280_DIG_P4_LSB_REG)));
	calibration.dig_P5 = ((int16_t)((readRegister(BME280_DIG_P5_MSB_REG) << 8) + readRegister(BME280_DIG_P5_LSB_REG)));
	calibration.dig_P6 = ((int16_t)((readRegister(BME280_DIG_P6_MSB_REG) << 8) + readRegister(BME280_DIG_P6_LSB_REG)));
	calibration.dig_P7 = ((int16_t)((readRegister(BME280_DIG_P7_MSB_REG) << 8) + readRegister(BME280_DIG_P7_LSB_REG)));
	calibration.dig_P8 = ((int16_t)((readRegister(BME280_DIG_P8_MSB_REG) << 8) + readRegister(BME280_DIG_P8_LSB_REG)));
	calibration.dig_P9 = ((int16_t)((readRegister(BME280_DIG_P9_MSB_REG) << 8) + readRegister(BME280_DIG_P9_LSB_REG)));

	calibration.dig_H1 = ((uint8_t)(readRegister(BME280_DIG_H1_REG)));
	calibration.dig_H2 = ((int16_t)((readRegister(BME280_DIG_H2_MSB_REG) << 8) + readRegister(BME280_DIG_H2_LSB_REG)));
	calibration.dig_H3 = ((uint8_t)(readRegister(BME280_DIG_H3_REG)));
	calibration.dig_H4 = ((int16_t)((readRegister(BME280_DIG_H4_MSB_REG) << 4) + (readRegister(BME280_DIG_H4_LSB_REG) & 0x0F)));
	calibration.dig_H5 = ((int16_t)((readRegister(BME280_DIG_H5_MSB_REG) << 4) + ((readRegister(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
	calibration.dig_H6 = ((uint8_t)readRegister(BME280_DIG_H6_REG));

	//111011X0
	//
	//writing 26 characters???
	//Set the oversampling control words.
	//config will only be writeable in sleep mode, so first insure that.
	writeRegister(BME280_CTRL_MEAS_REG, 0x00);


	//Set the config word
	dataToWrite = (settings.tStandby << 0x5) & 0xE0;
	dataToWrite |= (settings.filter << 0x02) & 0x1C;
	writeRegister(BME280_CONFIG_REG, dataToWrite);

	//Set ctrl_hum first, then ctrl_meas to activate ctrl_hum
	dataToWrite = settings.humidOverSample & 0x07; //all other bits can be ignored
	writeRegister(BME280_CTRL_HUMIDITY_REG, dataToWrite);

	//set ctrl_meas
	//First, set temp oversampling
	dataToWrite = (settings.tempOverSample << 0x5) & 0xE0;
	//Next, pressure oversampling
	dataToWrite |= (settings.pressOverSample << 0x02) & 0x1C;
	//Last, set mode
	dataToWrite |= (settings.runMode) & 0x03;
	//Load the byte
	writeRegister(BME280_CTRL_MEAS_REG, dataToWrite);

	return readRegister(0xD0);////Chip ID
}

//Strictly resets.  Run .begin() afterwards
void reset( void )
{
	writeRegister(BME280_RST_REG, 0xB6);

}

//****************************************************************************//
//
//  Pressure Section
//
//****************************************************************************//
float readFloatPressure( void )
{

	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of â€œ24674867â€� represents 24674867/256 = 96386.2 Pa = 963.862 hPa
	int32_t adc_P = ((uint32_t)readRegister(BME280_PRESSURE_MSB_REG) << 12) | ((uint32_t)readRegister(BME280_PRESSURE_LSB_REG) << 4) | ((readRegister(BME280_PRESSURE_XLSB_REG) >> 4) & 0x0F);

	int64_t var1, var2, p_acc;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)calibration.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calibration.dig_P5)<<17);
	var2 = var2 + (((int64_t)calibration.dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)calibration.dig_P3)>>8) + ((var1 * (int64_t)calibration.dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration.dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p_acc = 1048576 - adc_P;
	p_acc = (((p_acc<<31) - var2)*3125)/var1;
	var1 = (((int64_t)calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
	var2 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
	p_acc = ((p_acc + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7)<<4);

	return (float)p_acc / 256.0;

}

float readFloatAltitudeMeters( void )
{
	float heightOutput = 0;

	heightOutput = ((float)-45846.2)*(pow(((float)readFloatPressure()/(float)101325), 0.190263) - (float)1);
	return heightOutput;

}

float readFloatAltitudeFeet( void )
{
	float heightOutput = 0;

	heightOutput = readFloatAltitudeMeters() * 3.28084;
	return heightOutput;

}

//****************************************************************************//
//
//  Humidity Section
//
//****************************************************************************//
float readFloatHumidity( void )
{

	// Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of â€œ47445â€� represents 47445/1024 = 46. 333 %RH
	int32_t adc_H = ((uint32_t)readRegister(BME280_HUMIDITY_MSB_REG) << 8) | ((uint32_t)readRegister(BME280_HUMIDITY_LSB_REG));

	int32_t var1;
	var1 = (t_fine - ((int32_t)76800));
	var1 = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) * var1)) +
	((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)calibration.dig_H6)) >> 10) * (((var1 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)calibration.dig_H2) + 8192) >> 14));
	var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
	var1 = (var1 < 0 ? 0 : var1);
	var1 = (var1 > 419430400 ? 419430400 : var1);

	return (float)(var1>>12) / 1024.0;

}



//****************************************************************************//
//
//  Temperature Section
//
//****************************************************************************//

float readTempC( void )
{
	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of â€œ5123â€� equals 51.23 DegC.
	// t_fine carries fine temperature as global value

	//get the reading (adc_T);
	int32_t adc_T = ((uint32_t)readRegister(BME280_TEMPERATURE_MSB_REG) << 12) | ((uint32_t)readRegister(BME280_TEMPERATURE_LSB_REG) << 4) | ((readRegister(BME280_TEMPERATURE_XLSB_REG) >> 4) & 0x0F);

	//By datasheet, calibrate
	int64_t var1, var2;

	var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) *
	((int32_t)calibration.dig_T3)) >> 14;
	t_fine = var1 + var2;
	float output = (t_fine * 5 + 128) >> 8;

	output = output / 100;

	return output;
}

float readTempF( void )
{
	float output = readTempC();
	output = (output * 9) / 5 + 32;

	return output;
}

//****************************************************************************//
//
//  Utility
//
//****************************************************************************//


/*
void readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	//define pointer that will point to the external space
	uint8_t i = 0;
	char c = 0;

		Wire.beginTransmission(settings.I2CAddress);
		Wire.write(offset);
		Wire.endTransmission();


		// request bytes from slave device
		Wire.requestFrom(settings.I2CAddress, length);
		while ( (Wire.available()) && (i < length))  // slave may send less than requested
		{
			c = Wire.read(); // receive a byte as character
			*outputPointer = c;
			outputPointer++;
			i++;
		}

}

uint8_t readRegister(uint8_t offset)
{
	//Return value
	uint8_t result;
	uint8_t numBytes = 1;

		Wire.beginTransmission(settings.I2CAddress);
		Wire.write(offset);
		Wire.endTransmission();

		Wire.requestFrom(settings.I2CAddress, numBytes);
		while ( Wire.available() ) // slave may send less than requested
		{
			result = Wire.read(); // receive a byte as a proper uint8_t
		}

  return result;
}

int16_t readRegisterInt16( uint8_t offset )
{
	uint8_t myBuffer[2];
	readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
	int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);

	return output;
}
*/
void readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	}


int16_t readRegisterInt16( uint8_t offset )
{

	//combine two registers>?

	unsigned char  pucRegValue[2];
	if (I2C_IF_ReadFrom(BME280_DEV_ADDR, &offset, 1,

                   pucRegValue, 2) != 0){}

	//little endian
	int16_t	pi16RegValue =pucRegValue[0] | (uint16_t)pucRegValue[1] << 8;
	return pi16RegValue;

	/*big endian
	 *
	 * bytes[1] | (uint16_t)bytes[0] << 8
	 */
}
int writeRegister(uint8_t offset, uint8_t dataToWrite)
{
		//Write the byte
/*		Wire.beginTransmission(settings.I2CAddress);
		Wire.write(offset);
		Wire.write(dataToWrite);
		Wire.endTransmission();
	*/
		 unsigned char ucData[2];
		    //
		    // Select the register to be written followed by the value.
		    //
		    ucData[0] = offset;
		    ucData[1] = dataToWrite;
		    //
		    // Initiate the I2C write
		    //
		    if(I2C_IF_Write(BME280_DEV_ADDR,ucData,2,1) == 0)
		    {
		        return SUCCESS;
		    }
		    else
		    {
		        DBG_PRINT("I2C write failed\n\r");
		    }

		    return FAILURE;

}
