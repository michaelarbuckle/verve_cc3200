//*****************************************************************************
// GP2Y101drv.h - Defines and Macros for the GP2Y101 Humidity Presure Temperature Sensor interface.
//


#ifndef __GP2Y101_H__
#define __GP2Y101_H__


//*****************************************************************************
// ADC I2C address
//*****************************************************************************
#define GP2Y101_DEV_ADDR ADC_DEV_ADDR
#define GP2Y101_CHID_ID_NUM       0x00

//*****************************************************************************
// GP2Y101 ADC Data Register related macros
//*****************************************************************************
#define GP2Y101_CONFIG_REG   (0x01)
//select ADC channel 2
//#define GP2Y101_CONFIG_0 0xE3 //0b11100011

#define GP2Y101_CONFIG_0 0xE3 //0b11100011
//#define GP2Y101_CONFIG_1 0x83 //0b10000011
#define GP2Y101_CONFIG_1 0x03 //0b00000011

#define GP2Y101_READ_REG   (0x0)


// bit 15-8 0xD3
// 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1
//
// bit 15 flag bit for no effect = 0
// Bits 14-12 input selection:               channel 2 = 110
// 100 ANC0;   101 ANC1; 110 ANC2; 111 ANC3
// Bits 11 - 9  001 : FS = ±4.096V(1)
// Bit 8        0 : Continuous conversion mode
// Bits 7 - 5   100 : 128SPS (default)  000 8sps   101 250SPS 110 250SPS
// Bits 4    0 : Traditional comparator with hysteresis (default)
// Bits 3    0 : Active low (default)
// Bits 2    0 : Non-latching comparator (default)
// Bits 1 - 0 11 : Disable comparator (default)
//	writeBuf[1] =0b11010011 0b10000101;


#define GP2Y101_POWERUP_MODE 0
#define GP2Y101_CONTINUOS_MODE 1

//*****************************************************************************
// GP2Y101 Data Interpretation macros
//*****************************************************************************
#define RESOLUTION_8BIT         ((float)(1.999 / 127))  //+-2g
#define G_VAL                   ((float)9.7798)




//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
int GP2Y101Open();
void GPIOConfigureLEDDust();
void GPIOLEDDustOn();
void GPIOLEDDustOff();
int GP2Y101Read( int* bActive, float* fDust);
int GP2Y101ReadTestValues( int* bActive, float* fDust);
int GP2Y101Close();
int GP2Y101PowerUpComplete();
int GP2Y101PowerUp();

typedef struct sGP2Y101DataStruct
{
    //
    // boolean flag to indicate if this task is actively updating these data
    // fields.
    //
    int bActive;

    //
    // The most recent pressure reading from the sensor.
    //
    float fDust;

    int raw;
    float normalized;
    float calculated;

} sGP2Y101Data_t;

sGP2Y101Data_t g_GP2Y101Data;


#endif //  __GP2Y101_H__
