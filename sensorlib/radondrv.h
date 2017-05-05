//*****************************************************************************
// RADONDrv.h - Defines and Macros for the RADONDrv interface.
//

#ifndef __RADONDRV_H__
#define __RADONDRV_H__

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
#define SENSOR_BAUD_RATE 9600


//*****************************************************************************
// RADON Device UART address
//*****************************************************************************
#define RADON_UART_ADDR 0x72
#define SPACE  0x20    			//32
#define ZERO  0x30   			//48
#define LESS_THAN   0x3C   		//60
#define GREATER_THAN   0x3E  	//62
#define CR   0x0D 				//13
#define LF   0x0A  				//10
#define ASTRSK   0x2A  			//ascii-42



//*****************************************************************************
// RADON Device details
//*****************************************************************************
#define RADON_MANUFAC_ID       0x5449
#define RADON_DEVICE_ID        0x0067

//*****************************************************************************
//
// API Function prototypes
//
//*****************************************************************************
int RADONUARTInit();
int radonTest(char* logs);
int radonDrvOpen();
int radonDrvReset();
int radonDrvGetStatus(int *status);
int radonDrvGetRadon(int *status, double *pfRadon, char* logs);

//int radonDrvGetRadon(int *status, float *pfRadon);
int radonDrvSetMsgTypeToChar();
int radonDrvSetMsgTypeToHex();
int radonDrvSetRadonAsync(unsigned char interupt, void* callback);
int radonDrvAsncHandler(void* callback);//callback?

int radonDrvGetRadonTestValues(int *status, float *pfRadon);

typedef struct sRadonRDataStruct
{
    //
    // boolean flag to indicate if this task is actively updating these data
    // fields.
    //
    int bActive;

    //
    // The most recent pressure reading from the sensor.
    //
    double fRadon;

    char sRadon[6];

} sRadonData_t;

sRadonData_t g_RadonData;

//**b***************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************

#endif //  __RADONDRV_H__
