//*****************************************************************************
// S8NSAIRDrv.h - Defines and Macros for the S8NSAIRDrv interface.
//

#ifndef __S8NSAIRDRV_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************


/*
 *
 Coding system: 8-bit binary

 1 start bit: 1 start bit

 8 data bits, least significant bit first

 Data bits

No parity bit
1 stop bit for receiving
2 stop bits at transmission
The reason for the absence of parity and stop bit con

 *
 * *
 */

#define SENSOR UARTA0_BASE
#define SENSOR_PERIPH PRCM_UARTA0

//#define SENSOR UARTA1_BASE
//#define SENSOR_PERIPH PRCM_UARTA1
#define SENSOR_BAUD_RATE 9600


//*****************************************************************************
// S8NSAIR Device I2C address
//*****************************************************************************
#define S8NSAIR_UART_ADDR         0xFE//
#define S8NSAIR_UART_ADDR2        0x68 //or 0xFE


//*****************************************************************************
// S8NSAIR Register offset address
//*****************************************************************************
//sensair read command
 #define FUNC_CODE_READ_INPUT_REGISTER 0x04
 #define START_INPUT_REGISTER_A 0x00
 #define START_INPUT_REGISTER_B 0x03
 #define INPUT_REGISTER_COUNT_A 0x00
 #define INPUT_REGISTER_COUNT_B 0x01

#define S8NSAIR_MANUFAC_ID_REG_ADDR      0xFE
#define S8NSAIR_DEVICE_ID_REG_ADDR       0xFF

//*****************************************************************************
// S8NSAIR Device details
//*****************************************************************************
#define S8NSAIR_MANUFAC_ID       0x5449
#define S8NSAIR_DEVICE_ID        0x0067

//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
void UARTClear();
void checkCRC(char* logs, int cnt);
int S8NSAIRUARTInit();
int S8NSAIRUARTInitRx();
int S8NSAIRDrvCalibrate(char* logs);
int S8NSAIRDrvOpen();
int S8NSAIRDrvGetCO2(int *status, float *pfCO2, char* err);
int S8NSAIRDrvGetCO2Debug(int *status, float *pfCO2, char* logs);
int S8NSAIRDrvGetCO2TestValues(int *status, float *pfCO2);



typedef struct sS8NSAIRDataStruct
{
    //
    // boolean flag to indicate if this task is actively updating these data
    // fields.
    //
    int bActive;

    //
    // The most recent pressure reading from the sensor.
    //
    float fCO2;

    int raw;
    float normalized;
    char sCO2[6];

} sS8NSAIRData_t;

sS8NSAIRData_t g_S8NSAIRData;

//**b***************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************

#endif //  __S8NSAIRDRV_H__
