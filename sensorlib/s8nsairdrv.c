#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <math.h>
#include "hw_types.h"
#include "hw_memmap.h"
#include "prcm.h"
#include "pin.h"
#include "uart.h"
#include "utils.h"
#include "rom.h"
#include "rom_map.h"
//#include "uart_if.h"

#include "osi.h"

#include "sensor_api_common_types.h"
#include "verve_utils.h"

#include "s8nsairdrv.h"

//****************************************************************************
//                      GLOBAL VARIABLES                                   
//****************************************************************************

//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define FAILURE                 -1
#define SUCCESS                 0
#define RET_IF_ERR(Func)        {int iRetVal = (Func); \
                                 if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

//#define DBG_PRINT
//Report


//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS                          
//****************************************************************************
float ComputeCO2(long dVobject);
long int hexToInt(const char* hexstring);
void UART0_Handler();
int S8NSAIRDrvSendRcv(unsigned char* wBuf,int wCnt,unsigned char* rBuf, int sleep);

//****************************************************************************
//
//! Returns the value in the specified register
//!
//! \param ucRegAddr is the offset register address
//! \param pusRegValue is the pointer to the register value store
//! 
//! This function  
//!    1. Returns the value in the specified register
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************

//****************************************************************************
//
//! Initialize the CO2 sensor
//!
//! \param None
//! 
//! This function  
//!    1. Get the device manufacturer and version
//!    2. Add any initialization here
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
int S8NSAIRUARTInit()
{

	//UART-0 Initialization
	MAP_PRCMPeripheralReset(SENSOR_PERIPH);

	//configure the uart0 for DEVICEUART
	MAP_UARTConfigSetExpClk(SENSOR,MAP_PRCMPeripheralClockGet(SENSOR_PERIPH),
	9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	UART_CONFIG_PAR_NONE));
//SENSOR_BAUD_RATE
	//MAP_UARTDMADisable(CONSOLE1,UART_DMA_RX+UART_DMA_TX+UART_DMA_ERR_RXSTOP); //disable dma
//	MAP_UARTFIFODisable(SENSOR); //disable fifo

	//here we will configure the UART-1 interrupt based communication
	//MAP_UARTIntRegister(SENSOR,UART0_Handler); //enable interrupts
	//MAP_UARTIntEnable(SENSOR,UART_INT_DMARX | UART_INT_DMATX);


    return SUCCESS;
}


int S8NSAIRUARTInitRx()
{

	//UART-0 Initialization
	MAP_PRCMPeripheralReset(SENSOR_PERIPH);

	//configure the uart0 for DEVICEUART
	MAP_UARTConfigSetExpClk(SENSOR,MAP_PRCMPeripheralClockGet(SENSOR_PERIPH),
	9600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_TWO |
	UART_CONFIG_PAR_NONE));
//SENSOR_BAUD_RATE
	//MAP_UARTDMADisable(CONSOLE1,UART_DMA_RX+UART_DMA_TX+UART_DMA_ERR_RXSTOP); //disable dma
//	MAP_UARTFIFODisable(SENSOR); //disable fifo

	//here we will configure the UART-1 interrupt based communication
	//MAP_UARTIntRegister(SENSOR,UART0_Handler); //enable interrupts
	//MAP_UARTIntEnable(SENSOR,UART_INT_DMARX | UART_INT_DMATX);


    return SUCCESS;
}


int S8NSAIRDrvOpen(){


    return SUCCESS;
}
//****************************************************************************
//
//! Compute the CO2 value from the raw data.
//!
//! \param dCO2Ambient is the local CO2 capture data
//! 
//! This function  
//!    1. Computes the CO2 ppm from the dCO2Ambient values
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************
float ComputeCO2ppm(long dCO2Ambient)
{
    //
    // This algo is obtained from 
    // http://processors.wiki.ti.com/index.php/SensorTag_User_Guide
    // #IR_Temperature_Sensor
    //
    float tObj = dCO2Ambient*10.0;
    return tObj;
}



static uint16_t crc16(uint8_t *req, uint8_t req_length)
{
    uint8_t j;
    uint16_t crc;

    crc = 0xFFFF;
    while (req_length--) {
	crc = crc ^ *req++;
	for (j=0; j < 8; j++) {
	    if (crc & 0x0001)
		crc = (crc >> 1) ^ 0xA001;
	    else
		crc = crc >> 1;
	}
    }

    return (crc << 8  | crc >> 8);
}

void checkCRC(char* logs, int cnt)
{
	int ii = 0;
	uint8_t leng =5;
	unsigned char wBuf[6];
	char buf[6];
	wBuf[0]=0x68;
	wBuf[1]=0x44;
	wBuf[2]=0x00;
	wBuf[3]=0x08;
	wBuf[4]=0x02;

	uint16_t crci = crc16(wBuf,leng );

	 logs[cnt++]='<';

	 sprintf(buf,"%x",crci);
	 leng = strlen(buf);
	 for ( ii=0;ii < leng;ii++)		 //	 logs[logLen++]=(char)wChar;S
	 {
		 logs[cnt++] = buf[ii];
	 }
	 logs[cnt++]='>';

/*
	 wBuf[0]=0xFE;
	 wBuf[1]=0x04;
	 wBuf[2]=0x00;
	   wBuf[3] = 0x03;//START_INPUT_REGISTER_B;
	 	wBuf[4] = 0x00;//INPUT_REGISTER_COUNT_A;
	 	wBuf[5] = 0x01;//INPUT_REGISTER_COUNT_B;
	 	leng = 6;
	 crci = crc16(wBuf,leng );


		 sprintf(wBuf,"%x",crci);
		 leng = strlen(wBuf);
		 for ( ii=0;ii < leng;ii++)		 //	 logs[logLen++]=(char)wChar;S
		 {
			 logs[cnt++] = wBuf[ii];
		 }
*/
	 logs[cnt]='\0';

}



int S8NSAIRDrvCalibrate(char* logs)
{
	int retval =0;
	int ii = 0;
	int logLen = 0;
	int wCnt = 7;
	unsigned char rChar;
	int sleepDuration = 1000;
	unsigned char wBuf[8];
	unsigned char rBuf[8];

	//<FE> <06> <00> <00> <00> <00> <9D> <C5>
			wBuf[0]=0xFE;
			wBuf[1]=0x06;
			wBuf[2]=0x00;
			wBuf[3]=0x00;
			wBuf[4]=0x00;
			wBuf[5]=0x9D;
			wBuf[6]=0xC5;

			retval = S8NSAIRDrvSendRcv(wBuf,wCnt,rBuf,sleepDuration);

			while( (rChar = rBuf[ii++] )!= '\0'  && ii < 8)
			{
				logs[logLen++] = rChar;
			}

			sleepDuration = 2500; // > 2 seconds
			wCnt = 8;

			//<FE> <06> <00> <01> <7C> <06> <6C> <C7>
			wBuf[0]=0xFE;
			wBuf[1]=0x06;
			wBuf[2]=0x00;
			wBuf[3]=0x01;
			wBuf[4]=0x7C;
			wBuf[5]=0x06;
			wBuf[6]=0x6C;
			wBuf[7]=0xC7;

			retval = S8NSAIRDrvSendRcv(wBuf,wCnt,rBuf,sleepDuration);
			ii=0;
			while( (rChar = rBuf[ii++] )!= '\0'  && ii < 8)
			{
				logs[logLen++] = rChar;
			}

			//<FE> <03> <00> <00> <00> <01> <90> <05>
			wBuf[0]=0xFE;
			wBuf[1]=0x03;
			wBuf[2]=0x00;
			wBuf[3]=0x00;
			wBuf[4]=0x00;
			wBuf[5]=0x01;
			wBuf[6]=0x90;
			wBuf[7]=0x25;

			retval = S8NSAIRDrvSendRcv(wBuf,wCnt,rBuf,sleepDuration);

			ii=0;
			while( (rChar = rBuf[ii++] )!= '\0'  && ii < 8)
			{
				logs[logLen++] = rChar;
			}

			logs[logLen] = '\0';

	}
int S8NSAIRDrvSendRcv(unsigned char* wBuf,int wCnt,unsigned char* rBuf, int sleep)
{
    int sCnt =0;
	int ii = 0;
	for ( ii=0;ii < wCnt;ii++)
	{if (!UARTCharPutNonBlocking(SENSOR,  wBuf[ii]) ){//error
	}}
	while(UARTCharsAvail(SENSOR) == false)
	{
	 	osi_Sleep(1);sCnt++;
		if (sCnt > sleep)
		{
		}
	}
	ii = 0;
	while(UARTCharsAvail(SENSOR) && wCnt < 7)
	  {
			rBuf[ii++] = UARTCharGetNonBlocking(SENSOR);
	  }
}


//****************************************************************************
//
//! Get the temperature value
//!
//! \param pfCurrTemp is the pointer to the temperature value store
//! 
//! This function  
//!    1. Get the sensor voltage reg and ambient temp reg values
//!    2. Compute the temperature from the read values
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************


int S8NSAIRDrvGetCO2TestValues(int *bActive, float *pfCO2)
{/*
	g_S8NSAIRData.bActive =	-1.0;

	g_S8NSAIRData.fCO2 =	414.0;
	*bActive = g_S8NSAIRData.bActive;
	*pfCO2 =  g_S8NSAIRData.fCO2;
	*/
	return 0;
}
/*
void hexToString(unsigned char hx, char* str)
{
sprintf(str,"%02x",hx);
}
*/
/*
extern void UARTConfigGetExpClk(unsigned long ulBase, unsigned long ulUARTClk,
                                unsigned long *pulBaud,
                                unsigned long *pulConfig);
*/


void UARTClear(){

while(UARTCharsAvail(SENSOR) )
  {
	unsigned char t = UARTCharGet(SENSOR);
  }
 UARTRxErrorClear(SENSOR);

}
void checkError(unsigned char retCode, char* err, int cnt)
{
						if ( (retCode & 0b00000100) > 0)
						{
							err[cnt++]='2';
						}
						if ( (retCode & 0b00001000) > 0)
							{
								err[cnt++]='3';
							}
						if ( (retCode & 0b00010000) > 0)
							{
								err[cnt++]='4';
							}
						if ( (retCode & 0b00100000) > 0)
							{
								err[cnt++]='5';
							}

						if ( (retCode & 0b01000000) > 0)
							{
								err[cnt++]='6';

							}
}

/*
//2 4 Algorithm Error.

 & 0b0010000
0 1 Fatal Error
1 2 Reserved
2 4 Algorithm Error.		(retCode & 0b00000100) > 0
3 8 Output Error  			0b00000100
4 16 Self-Diagnostic Error. 0b00001000
5 32 Out Of Range Error  	0b00010000
6 64 Memory Error        	0b00100000
128  						0b01000000
*/

int S8NSAIRDrvGetCO2(int *status, float *pfCO2, char* err)
{
	int ii = 0;
	int iii = 0;
	int logLen = 0;
int dCount = 7;
/**/
int rCnt = 0;
int sCnt = 0;
long rLong;
int valueLength = 0;
uint16_t value = 0;
unsigned char rChar;
unsigned char rTmp;
unsigned char rBuf[4];

unsigned char wBuf[7];

	wBuf[0]=0xFE;
	wBuf[1]=0x44;
	wBuf[2]=0x00;
	wBuf[3]=0x08;
	wBuf[4]=0x02;
	wBuf[5]=0x9F;
	wBuf[6]=0x25;


for ( ii=0;ii < dCount;ii++)
{if (!UARTCharPutNonBlocking(SENSOR,  wBuf[ii]) )
{
     err[logLen++] = 'e';
}
}//for


while(UARTCharsAvail(SENSOR) == false)
 {
 	osi_Sleep(1);
	sCnt++;

	//dCount++;
	if (sCnt > 30000){

		err[logLen++]=' ';
		err[logLen++]='t';
		err[logLen++]='o';
		err[logLen++]=' ';

			err[logLen]='\0';

			return SUCCESS;
	}

 }


while(UARTCharsAvail(SENSOR) && rCnt < 7)
  {
		rLong = UARTCharGetNonBlocking(SENSOR);
		if (rLong != -1)
		{
			rChar = (unsigned char)rLong;
			//skip prefix

			if( (rCnt == 0 && !(rChar == 0xFE)) || (rCnt == 1 && !(rChar == 0x44)) )
			{
				//error
			}
			if (rCnt == 2  )
			{
				valueLength = (int)rLong;

			}
			if (rCnt == 3)
			{
				//msb of value
				rTmp =rChar;
			} else if (rCnt == 4)
			{

				//lsb of value
				value = rTmp << 8 | rChar;

				*pfCO2 = (float)value;
			}

			sprintf(wBuf," %x",rChar);
			sCnt = strlen(wBuf);
			for ( iii=0;iii < sCnt;iii++)		 //	 logs[logLen++]=(char)wChar;S
			{
				err[logLen++] = wBuf[iii];
			}
		}
	rCnt++;
	osi_Sleep(5);

  }

rLong= UARTRxErrorGet(SENSOR);
if (rLong > 0)
{
	 err[logLen++] = 'E';

	sprintf(rBuf,"%ld",rLong);
	 dCount = strlen(rBuf);
	 for ( ii=0;ii < dCount;ii++)		 //	 logs[logLen++]=(char)wChar;S
	 {
		 err[logLen++] = rBuf[ii];
	 }
	 UARTRxErrorClear(SENSOR);
}


err[logLen++]='\0';

return SUCCESS;

}



int S8NSAIRDrvGetCO2Debug(int *status, float *pfCO2, char* logs)
{

	return SUCCESS;

}



//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************




void UART0_Handler()
{


}



void charToHex(const char* word, char* outword)
{
int len = sizeof(word);
int i;
	    for(i = 0; i<len; i++){
	        sprintf(outword+i*2, "%02X", word[i]);
	    }
}

long int hexToInt(const char* hexstring)
{
	int number = (int)strtol(hexstring, NULL, 16);

     return number;

}
/*
char *getln()
{
    char *line = NULL, *tmp = NULL;
    size_t size = 0, index = 0;
    int ch = EOF;

    while (ch) {
        ch = getc(stdin);

        // Check if we need to stop.
        if (ch == EOF || ch == '\n')
            ch = 0;

        // Check if we need to expand.
        if (size <= index) {
            size += CHUNK;
            tmp = realloc(line, size);
            if (!tmp) {
                free(line);
                line = NULL;
                break;
            }
            line = tmp;
        }

        // Actually store the thing.
        line[index++] = ch;
    }

    return line;
}
*/
//to determine whether it is TX or RX interrupt
//UARTIntStatus(unsigned long ulBase, tBoolean bMasked)

/*
 All registers are 16 bit word


//sensair read command
 #define FUNC_CODE_READ_INPUT_REGISTER 0x04
 #define START_INPUT_REGISTER_A 0x00
 #define START_INPUT_REGISTER_B 0x03
 #define INPUT_REGISTER_COUNT_A 0x00
 #define INPUT_REGISTER_COUNT_B 0x01
 #define



	/*
	wBuf[0]=254;//0xFE;
	wBuf[1]=4;//0x04;
	wBuf[2]=0;//0x00;
	wBuf[3]=0;//0x00;
	wBuf[4]=0;//0x00;
	wBuf[5]=1;//0x01;
	wBuf[6]=37;//0x25;
	wBuf[7]=197;//0xC5;
	*/
	/*
	wBuf[0]=0xFE;
	wBuf[1]=0x04;
	wBuf[2]=0x00;
	wBuf[3]=0x00;
	wBuf[4]=0x00;
	wBuf[5]=0x01;
	wBuf[6]=0x25;
	wBuf[7]=0xC5;
	*/
	/*
	wBuf[0]=0xFE;
	wBuf[1]=0x04;
	wBuf[2]=0x00;
	  wBuf[3] = 0x03;//START_INPUT_REGISTER_B;
		wBuf[4] = 0x00;//INPUT_REGISTER_COUNT_A;
		wBuf[5] = 0x01;//INPUT_REGISTER_COUNT_B;
		wBuf[6] = 0xD5;
		wBuf[7] = 0xC5;

	*/

/*
    wBuf[0]=0x68;
	wBuf[1]=0x44;
	wBuf[2]=0x00;
	wBuf[3]=0x08;
	wBuf[4]=0x02;
	wBuf[5]=0x25;//low byte
	wBuf[6]=0x80;//hsb



CO2 read sequence:
The sensor is addressed as “Any address” (0xFE).
We read CO2 value from IR4 using “Read input registers” (function code 04). Hence,
Starting address will be 0x0003 (register number-1) and Quantity of registers 0x0001.
CRC calculated to 0xC5D5 is sent with low byte first.
We assume in this example that by sensor measured CO2 value is 400ppm*.
Sensor replies with CO2 reading 400ppm (400 ppm = 0x190 hexadecimal).

Master Transmit:
<FE> <04> <00> <03> <00> <01> <D5> <C5>
Slave Reply:
<FE> <04> <02> <01> <90> <AC> <D8>


Sensor status read sequence:
The sensor is addressed as “Any address” (0xFE).
We read status from IR1 using “Read input registers” (function code 04). Hence,
Starting address will be 0x0000 (register number-1) and Quantity of registers 0x0001.
CRC calculated to 0xC525 is sent with low byte first.
Sensor replies with status 0.
Master Transmit:
<FE> <04> <00> <00> <00> <01> <25> <C5>
Slave Reply:
<FE> <04> <02> <00> <00> <AD> <24>


Exceptions

Invalid function code.
 Invalid data address (requested register doesn’t exist in given device).
 Invalid data.
 Error in execution of requested function.

Show what you have - are you planning to do the reception of characters sitting in a loop (this is called "polled" or sometimes "synchronously") or are you planning to have the characters received by a UART interrupt (this is known as "asynchronously"). In either case you will ultimately have a single function that returns a character at a time. Maybe called getchar()?. To receive a string you keep processing getchar() until you see an end of line marker has arrived. That can be denoted by either carriage return (0x0D also '\r') or line feed (0z0A or '\n'). So something like:
s
*/
