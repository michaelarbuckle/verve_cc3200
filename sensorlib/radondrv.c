#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "uart_if.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "utils.h"
#include "prcm.h"
#include "pin.h"
#include "uart.h"
#include "rom.h"
#include "rom_map.h"
#include "common.h"

#include "osi.h"

#include "sensor_api_common_types.h"

#include "radondrv.h"

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
/*
	unsigned char RESET = 'k';
	unsigned char READ = 'r';
	unsigned char CHAR = 'c';
	unsigned char NUM = 'n';
*/

//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************
double ComputeRadon(long dVobject);

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

/*
The only data sent out by the system (when requested or every hour) is the Radon
concentration level expressed in Becquerel/m³ with a maximum of 65’000 Bq/m³.
The format of the sent value is the following:
*/

//interupt handler for radon

int radonDrvGetRadon(int *status, double *pfRadon, char* logs)
{

	/*
	unsigned char SPACE = 0x20;   //32
	unsigned char ZERO = 0x30;   //48
	unsigned char LESS_THAN = 0x3C;   //60
	unsigned char GREATER_THAN = 0x3E;  //62
	unsigned char CR = 0x0D; //13
	unsigned char LF = 0x0A; //10
	unsigned char ASTRSK = 0x2A; // ascii-42
*/

	//t0x3c 0x20 0x0 0x0 0x0 0x30 0x3e 0xd 0xa
	uint16_t val = 9999;
	unsigned char LSB =0x00;
	unsigned char MSB =0x00;



	tBoolean lsb = false;
	tBoolean open = false;
	tBoolean complete = false;
	int iii = 0;
	int ii = 0;
	int logLen = 0;
int dCount = 1;
/**/
long rLong = 0;
int rCnt = 0;
int sCnt = 0;
unsigned char wChar;
unsigned char rBuf[12];
/**/
unsigned char wBuf[1];
wBuf[0] = 'r';// 0x72;//ascii 114 or char='r'


for ( ii=0;ii < dCount;ii++)
{


if (UARTCharPutNonBlocking(SENSOR,  wBuf[ii]) )
{
	logs[logLen++]='t';
} else
{
	logs[logLen++]='f';
}

}//for


while(UARTCharsAvail(SENSOR) == false) {
 	osi_Sleep(1);
 	//	UtilsDelay(1);
 		//dCount++;
 		if (sCnt < 25000){
 		sCnt++;
 		} else
 		{

 			logs[logLen++]='t';
 			logs[logLen++]='o';
 			logs[logLen++]='\0';
 			return FAILURE;
 		}

 }



ii = 0;
if(UARTCharsAvail(SENSOR) )
  {
	while (!complete && ii < 12 )
	{
		rBuf[ii] = UARTCharGet(SENSOR);

		if (rBuf[ii] == ASTRSK)
		{
			logs[logLen++]='i';
			logs[logLen++]='n';
			logs[logLen++]='v';
			logs[logLen++]='\0';

			return FAILURE;
		}
		else	if (rBuf[ii] == GREATER_THAN)
		{
			open = false;
		}
		else if (rBuf[ii] == LF)
		{
			complete = true;
		}
		if (open == true || open == false)
		{
			sprintf(wBuf," %x",rBuf[ii]);
			sCnt = strlen(wBuf);
			for ( iii=0;iii < sCnt;iii++)		 //	 logs[logLen++]=(char)wChar;S
			{
				logs[logLen++] = wBuf[iii];

				if (wBuf[iii] == SPACE)
				{

				}/* else if (wBuf[iii] == ZERO)
				{
					val = 0;
				} */else
				{


				if (lsb == true)
				{
					LSB = wBuf[iii];
					if (!(MSB == 0x0  && LSB == 0x0))
					{

						  val = MSB << 8 | LSB;
						  *pfRadon = (float)val;
						  *status = -1;
					}
					lsb = false;
				}
				else
				{

					MSB = wBuf[iii];
					lsb = true;
				}

				}
			}

		}
		if (rBuf[ii] == LESS_THAN)
		{
			open = true;
		}
	}
	//

  }
	  //check for error

	  rLong= UARTRxErrorGet(SENSOR);
	  if (rLong > 0)
	  {
		  logs[logLen++] = 'E';

		  sprintf(rBuf,"%ld",rLong);
		  dCount = strlen(rBuf);
		  for ( ii=0;ii < dCount;ii++)		 //	 logs[logLen++]=(char)wChar;S
		  {
			  logs[logLen++] = rBuf[ii];
		  }
		  UARTRxErrorClear(SENSOR);
	  }



logs[logLen++]=' ';
logs[logLen++]='r';
logs[logLen++]='\0';

return SUCCESS;

}


int radonDrvSetRadonAsync(unsigned char interupt, void* callback)
{
return SUCCESS;
}
int radonDrvAsncHandler(void* callback)//callback?
{
return SUCCESS;
}
//****************************************************************************
//
//! Initialize the radon sensor
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
int
radonUARTInit()
{

	//UART-0 Initialization
	MAP_PRCMPeripheralReset(SENSOR_PERIPH);

	//configure the uart0 for DEVICEUART
	MAP_UARTConfigSetExpClk(SENSOR,MAP_PRCMPeripheralClockGet(SENSOR_PERIPH),
	SENSOR_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	UART_CONFIG_PAR_NONE));

	//MAP_UARTDMADisable(CONSOLE1,UART_DMA_RX+UART_DMA_TX+UART_DMA_ERR_RXSTOP); //disable dma
	//MAP_UARTFIFODisable(SENSOR); //disable fifo

	//here we will configure the UART-1 interrupt based communication
	//MAP_UARTIntRegister(SENSOR,UART0_Handler); //enable interrupts
	//MAP_UARTIntEnable(SENSOR,UART_INT_DMARX | UART_INT_DMATX);


    return SUCCESS;
}
//****************************************************************************
//
//! Initialize radon sensor UART channel
//!
//! \param
//!
//! This function
//!    \
//!
//! \return 0: Success, < 0: Failure.
//
//****************************************************************************

int radonDrvOpen(){

    return SUCCESS;
}
//****************************************************************************
//
//! Compute the radon value from the sensor reading.
//!
//! \param dVRadon is the sensor voltage value
//!
//! This function
//!    1. Computes the radon from parameter values
//!
//! \return double  in units Bq/m³
//
//****************************************************************************

int radonDrvGetRadonTestValues(int *bActive, float *pfRadon)
{

	g_RadonData.bActive = -1;
	g_RadonData.fRadon =	56.0;
	*bActive = g_RadonData.bActive;
	*pfRadon =  g_RadonData.fRadon;
	return SUCCESS;
}



int radonDrvSetMsgTypeToChar()
{
return SUCCESS;
}
int radonDrvSetMsgTypeToHex()
{

unsigned char NUM = 'n';


int ii = 0;
int logLen = 0;
int dCount = 1;
int sCnt = 0;
unsigned char wBuf[1];
wBuf[0] = NUM;// 0x72;//ascii 114 or char='r'

if (UARTCharPutNonBlocking(SENSOR, wBuf[0]) )
{
}


while(UARTCharsAvail(SENSOR) == false) {
	osi_Sleep(1);
	//	UtilsDelay(1);
		//dCount++;
		if (sCnt < 5000){
		sCnt++;
		} else
		{

			return FAILURE;
		}

}



ii = 0;
while(UARTCharsAvail(SENSOR) )
{

	wBuf[0] = UARTCharGet(SENSOR);

}

return SUCCESS;

}

/*
That can be changed by sending the ‘c’ (ASCII: 99d,
63h) or the ‘n’ (ASCII: 110d, 6Eh) character respectively for ‘char’ and
‘numeric’ format.
It is achievable
by sending the ‘k’ (ASCII: 107d, 6Bh) character. Still, no ACK or positive
feedback is sent back.


*/
