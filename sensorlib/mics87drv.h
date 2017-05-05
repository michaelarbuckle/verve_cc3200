//*****************************************************************************
// MICS87drv.h - Defines and Macros for the MICS87 Humidity Presure Temperature Sensor interface.
//


#ifndef __MICS87_H__
#define __MICS87_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************

//*****************************************************************************
// ADC I2C address
//*****************************************************************************
#define MICS87_DEV_ADDR ADC_DEV_ADDR
#define MICS87_CHID_ID_NUM       0x00



//*****************************************************************************
// MICS87 ADC Data Register related macros
//*****************************************************************************
#define MICS87_CONFIG_REG   (0x01)
//select ADC channel 1
#define MICS87_CONFIG_0 0xC3   //15-8  0xD3  0b11000011    start conversion on ADC 0, 4V, No continuos Conversion
#define MICS87_CONFIG_1 0x03   //7-0 0b00000011         tried: 0b10000011

#define MICS87_READ_REG   (0x0)

// bit 15-8 0xD3
// 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1
// bit 15 flag bit for no effect = 0
// Bits 14-12 input selection:
// 100 ANC0;   101 ANC1; 110 ANC2; 111 ANC3
// Bits 11 - 9  001 : FS = ±4.096V(1)
// Bit 8        0 : Continuous conversion mode   1: No continuos conversion
// Bits 7 - 5   100 : 128SPS (default)    samples per second ??? possibly 8SPS???
// Bits 4    0 : Traditional comparator with hysteresis (default)
// Bits 3    0 : Active low (default)
// Bits 2    0 : Non-latching comparator (default)
// Bits 1 - 0 11 : Disable comparator (default)
//	writeBuf[1] =0b11010011 0b10000101;


#define MICS87_POWERUP_MODE 0
#define MICS87_CONTINUOS_MODE 1

//*****************************************************************************
// MICS87 Data Interpretation macros
//*****************************************************************************
#define RESOLUTION_8BIT         ((float)(1.999 / 127))  //+-2g
#define G_VAL                   ((float)9.7798)

//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
int MICS87Open();
int MICS87Read( int* bActive, float* fVOC, char* logs);
int MICS87ReadTestValues( int* bActive, float* fVOC);
int MICS87Close();
int MICS87PowerUpComplete();
int MICS87PowerUp();




typedef struct sMICS87DataStruct
{
    //
    // boolean flag to indicate if this task is actively updating these data
    // fields.
    //
    int bActive;

    //
    // The most recent pressure reading from the sensor.
    //
    float fVOC;


    //
    // Most recent temperature reading from the sensor.
    //
    float fTemperature;


    int raw;
    float normalized;
    float calculated;


} sMICS87Data_t;

sMICS87Data_t g_MICS87Data;



//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************

#endif //  __MICS87_H__
