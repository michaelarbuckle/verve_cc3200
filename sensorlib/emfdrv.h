//*****************************************************************************
// EMFdrv.h - Defines and Macros for the EMF Humidity Presure Temperature Sensor interface.
//


#ifndef __EMF_H__
#define __EMF_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
//*****************************************************************************
// ADC I2C address
//*****************************************************************************
#define EMF_DEV_ADDR ADC_DEV_ADDR
#define EMF_CHID_ID_NUM       0x00

//*****************************************************************************
// EMF ADC Data Register related macros
//*****************************************************************************
#define EMF_CONFIG_REG   (0x01)
//select ADC channel 1
#define EMF_CONFIG_0 0xD2 //0b11010010  D2
#define EMF_CONFIG_0 0xD3 //0b11010011  D3
//#define EMF_CONFIG_0 0xD2 //0b11010010  D2
//#define EMF_CONFIG_1 0x83 //0b10000011  128 SPS
#define EMF_CONFIG_1 0xA3 //0b10100011  250
//#define EMF_CONFIG_1 0xC3 //0b11000011  475

//#define EMF_CONFIG_1 0x03 //0b0000 0011


#define EMF_READ_REG   (0x0)

// bit 15-8 0xD3S
// 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1
// bit 15 flag bit for no effect = 0
// Bits 14-12 input selection:
// 100 ANC0;   101 ANC1; 110 ANC2; 111 ANC3
// Bits 11 - 9  001 : FS = ±4.096V(1)
// Bit 8        0 : Continuous conversion mode
// Bits 7 - 5   100 : 128SPS (default)   000 8sps   101 250SPS 110 250SPS
// Bits 4    0 : Traditional comparator with hysteresis (default)
// Bits 3    0 : Active low (default)
// Bits 2    0 : Non-latching comparator (default)
// Bits 1 - 0 11 : Disable comparator (default)
//	writeBuf[1] =0b11010011 0b10000101;


#define EMF_POWERUP_MODE 0
#define EMF_CONTINUOS_MODE 1

//*****************************************************************************
// EMF Data Interpretation macros
//*****************************************************************************
#define RESOLUTION_8BIT         ((float)(1.999 / 127))  //+-2g
#define G_VAL                   ((float)9.7798)

//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
int EMFOpen();
int EMFRead( int* bActive, float* fV_m);
int EMFReadTest( int* bActive, float* fV_m);

float convertV_mTomW_cm(float fV_m);
int EMFClose();
int EMFPowerUpComplete();
int EMFPowerUp();

/*
 * When measuring the generally low levels of background RF radiation, you can use a small unit, such as V/m.
 * When measuring the relatively large amount of radiation that leaks from a microwave oven, you can use a larger unit, such as mW/cm2.
 *
 */


typedef struct sEMFDataStruct
{
    //
    // boolean flag to indicate if this task is actively updating these data
    // fields.
    //
    int bActive;

    //
    // The most recent intensity reading from the sensor.
    //
    float fIntensity;


    int raw;
    float normalized;
    float calculated;

} sEMFData_t;

sEMFData_t g_EMFData;


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************


#endif //  __EMF_H__
