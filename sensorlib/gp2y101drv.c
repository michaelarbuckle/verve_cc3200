//*****************************************************************************
// GP2Y101drv.c - Accelerometer sensor driver APIs.
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "hw_types.h"
#include "hw_memmap.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio.h"

#include "common.h"
#include "ads1115.h"
#include "gp2y101drv.h"
// Common interface includes
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

#define GPIO_LED_DUST 18


//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                          
//****************************************************************************
const float dustIncrement = .29;//.1/.35;   //sensitivity  is .35 V/.1 mg/m^3

const float range = 3.4;
const float lowZero = .9;
const float highZero =  1.5;



int measurePin = 0;

int ledPower = 3;


int samplingTime = 280; //driving pulse definition
int deltaTime = 40;
int sleepTime = 9680;

const int numReadings = 100; //Number of samples
int index = 0;                  // the index of the current reading
float DustReadings[100];  // the readings from the dustsensor
float totalDust = 0; // the running total Dust
float averageDust = 0; //calculated average

float Vref=5;   // max Voltage of the ADC
float voMeasured = 0; //Dust sensor reading byte
float calcVoltage = 0;  //Dust sensor converted in Volt
float dustDensity = 0;  //Dust density in micro gramm / m3
float Voc=0.65;          //No dust value in volt


unsigned int g_uiLEDDustPort = 0;
unsigned char g_ucLEDDustPin;
//GPIOA3_BASE, 0x10

//configure
void GPIOConfigureLEDDust()
{
	//GPIO_IF_GetPortNPin(GPIO_LED_DUST, &g_uiLEDDustPort, &g_ucLEDDustPin);
}


void GPIOLEDDustOn()
//void GPIOLEDDustOff()
{
//	 MAP_GPIOPinWrite(GPIOA3_BASE,GPIO_PIN_6,GPIO_PIN_6);
	 MAP_GPIOPinWrite(GPIOA3_BASE,GPIO_PIN_4,GPIO_PIN_4);

//#define GPIO_PIN_4 0x10
}
void GPIOLEDDustOff()
//void GPIOLEDDustOn()
{
	MAP_GPIOPinWrite(GPIOA3_BASE,GPIO_PIN_4,0);
}

int GP2Y101PowerUp()
{
	g_GP2Y101Data.bActive = GP2Y101_POWERUP_MODE;
//First minute after Power-on
//Functional Test Mode
	return SUCCESS;
}


int
GP2Y101PowerUpComplete()
{
	g_GP2Y101Data.bActive = GP2Y101_CONTINUOS_MODE;
	return SUCCESS;
}
//****************************************************************************
//
//! Initialize the GP2Y101 accelerometer device with defaults
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
GP2Y101Open()
{
    unsigned char ucRegVal;
    //
    // Read the CHIP ID NUM
    //
    RET_IF_ERR(GetADCRegisterValue(GP2Y101_CHID_ID_NUM, &ucRegVal));
    DBG_PRINT("CHIP ID: 0x%x\n\r", ucRegVal);

    return SUCCESS;
}

//****************************************************************************
//
//! Place the GP2Y101 accelerometer device to standby
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
GP2Y101Close()
{
    return SUCCESS;
}


float computeDust(float analogValue)
{
	float dust = (analogValue) * (dustIncrement);

    return dust;
}
//****************************************************************************
//
//! Get the dust data readings
//!
//! \param pfAccX pointer to the AccX store
//! 
//! This function  
//!    1. Reads the data registers over I2C.
//!    2. Applies the range conversion to the raw values
//!
//! \return 0: Success, < 0: Failure.
//
//********in********************************************************************



int GP2Y101ReadValue( int* bActive, float* fDust)
{

	uint8_t writeBuf[3];
	uint8_t readBuf[2];
	uint8_t configPtr[1];

	  configPtr[0] = GP2Y101_CONFIG_REG;

	  writeBuf[0] =	  GP2Y101_CONFIG_REG;
	  writeBuf[1] =GP2Y101_CONFIG_0;
	  writeBuf[2] =GP2Y101_CONFIG_1;

	  RET_IF_ERR(I2C_IF_Write(GP2Y101_DEV_ADDR,writeBuf,3,1));


	  readBuf[0] = 0;   //set conversion register to 0 for reading
	  readBuf[1] = 0;   //bookkeeping

	  // wait for conversion complete
	  // checking bit 15
	  do {
		  RET_IF_ERR(I2C_IF_ReadFrom(GP2Y101_DEV_ADDR,configPtr, 1,
		                  readBuf, 2) );
		  //delay??
	  }
	  while (((readBuf[0] & 0x80) == 0) );


	  // read conversion register
	  // write register pointer first
	  readBuf[0] = 0;   // conversion register is 0
	//  readBuf[1] = 0;
	  configPtr[0] = 0;

	  RET_IF_ERR(I2C_IF_Write(GP2Y101_DEV_ADDR,readBuf,1,1));


	  // read 2 bytes
	  RET_IF_ERR(I2C_IF_ReadFrom(GP2Y101_DEV_ADDR,configPtr, 1,
	                  readBuf, 2) );

	  // convert display results
	  uint16_t  val = readBuf[0] << 8 | readBuf[1];
 	  g_GP2Y101Data.raw = val;

		//normalize
	  float analog = (float)val*4.096/32767.0;

	  g_GP2Y101Data.normalized = analog;


	  *fDust =  computeDust(analog);
	  *bActive  = SUCCESS;

	  return SUCCESS;
}


int SamplingDust ( int* bActive, float* fDust);

int GP2Y101Read( int* bActive, float* fDust)
{
	int retval =0;
    GPIOLEDDustOn();
    osi_Sleep(samplingTime);

    retval = GP2Y101ReadValue(  bActive,  fDust); // read the dust value

    osi_Sleep(deltaTime);
    GPIOLEDDustOff();
	return retval;
}

int GP2Y101ReadAvg( int* bActive, float* fDust)
{

    GPIOLEDDustOn();
    osi_Sleep(samplingTime);

//  digitalWrite(ledPower,LOW); // power on the LED
//  delayMicroseconds(samplingTime);

  int retval = GP2Y101ReadValue(  bActive,  fDust); // read the dust value

  voMeasured = *fDust;

//voMeasured   = analogRead(measurePin);

    osi_Sleep(deltaTime);
    GPIOLEDDustOff();
//    GPIO_IF_LedOff(MCU_RED_LED_GPIO)S;
    osi_Sleep(samplingTime);

//  delayMicroseconds(deltaTime);
// digitalWrite(ledPower,HIGH); // turn the LED off
// delayMicroseconds(sleepTime);

  // 0 - 3.3V mapped to 0 - 1023 integer values
  // recover voltage
  calcVoltage = voMeasured * (Vref / 1024);


  // linear eqaution taken from http://www.howmuchsnow.com/arduino/airquality/
  // Chris Nafis (c) 2012
  if (calcVoltage<Voc)
   {
      calcVoltage=Voc;
   }
  dustDensity = (0.17 * (calcVoltage -Voc))*1000; //Dust density calculation
  totalDust = totalDust - DustReadings[index];  //removing the last reading
  DustReadings[index]=dustDensity;  //array storage of dust density
  totalDust = totalDust + DustReadings[index];  //Total dust for average calculation

  index = index + 1;    //Increment the array index
  if (index >= numReadings) //resting array index after 100 values
    {
    index = 0;
    }

    //Average calculation
    averageDust = totalDust / numReadings;

  osi_Sleep(100); //delay to the next measure
  //g_GP2Y101Data.normalized = averageDust;

  return SUCCESS;

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
