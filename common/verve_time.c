#include "verve_time.h"

#include <stdbool.h>
//#include <stdlib.h>
//#include <time.h>


/* XDCtools Header files */
/*#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/System.h>
*/
/* SYS/BIOS Headers */
//#include <ti/sysbios/BIOS.h>
//#include <ti/sysbios/knl/Task.h>

/* Peripheral Headers */
//#include <ti/drivers/WiFi.h>

/* SimpleLink Wi-Fi Host Driver Header files */
#include <simplelink.h>
#include <netapp.h>
#include <netcfg.h>
#include <fs.h>
#include <socket.h>

//****************************************************************************
//                          LOCAL DEFINES
//****************************************************************************

#define TASK_PRIORITY       (1)
#define SSID_LEN_MAX        32
#define BSSID_LEN_MAX       6
#define SUCCESS             0
#define READ_SIZE           1450
#define MAX_BUFF_SIZE       1460

//*****************************************************************************
//                         APPLICATION DEFINES
//*****************************************************************************


//*****************************************************************************
//                       SNTP CONFIGURATION DEFINES
//*****************************************************************************

#define TIME2013                3565987200u      /* 113 years + 28 days(leap) */
#define YEAR2013                2013
#define SEC_IN_MIN              60
#define SEC_IN_HOUR             3600
#define SEC_IN_DAY              86400

#define SERVER_RESPONSE_TIMEOUT 10
#define GMT_DIFF_TIME_HRS       10
#define GMT_DIFF_TIME_MINS      0

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus = 0;//SimpleLink Status
unsigned long  g_ulDestinationIP; // IP address of destination server
unsigned long  g_ulGatewayIP = 0; //Network Gateway IP address
unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
unsigned char g_buff[MAX_BUFF_SIZE+1];
long bytesReceived = 0; // variable to store the file size
int g_iSockID;
unsigned long g_ulElapsedSec;
short g_isGeneralVar;
unsigned long g_ulGeneralVar;
unsigned long g_ulGeneralVar1;
char g_acTimeStore[30];
char *g_pcCCPtr;
unsigned short g_uisCCLen;

SlSockAddr_t sAddr;
SlSockAddrIn_t sLocalAddr;

const char g_acSNTPserver[25] = "ntp.verve.local"; // Telstra SNTP Server

//*****************************************************************************
//                  Globals for Date and Time
//*****************************************************************************

// Tuesday is the 1st day in 2013 - the relative year
const char g_acDaysOfWeek2013[7][3] = {{"Tue"},{"Wed"},{"Thu"},{"Fri"},{"Sat"},{"Sun"},{"Mon"}};
const char g_acMonthOfYear[12][3] = {{"Jan"},{"Feb"},{"Mar"},{"Apr"},{"May"},{"Jun"},{"Jul"},{"Aug"},{"Sep"},{"Oct"},{"Nov"},{"Dec"}};
const char g_acNumOfDaysPerMonth[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
const char g_acDigits[] = "0123456789";

//*****************************************************************************
// Globals used by sockets and simplelink
//*****************************************************************************
unsigned char buffer[UDPPACKETSIZE];
bool deviceConnected = false;
bool ipAcquired = false;


char returnPacket[128];

typedef struct
{
    unsigned long  ipV4;
    unsigned long  ipV4Mask;
    unsigned long  ipV4Gateway;
    unsigned long  ipV4DnsServer;
}_NetCfgIpV4Args_t;


/* variable to be read by GUI Composer */
int count = 0;

//*****************************************************************************
//                         UTILITIES
//*****************************************************************************

//*****************************************************************************
//
//! itoa
//!
//!    @brief  Convert integer to ASCII in decimal base
//!
//!     @param  cNum is input integer number to convert
//!     @param  cString is output string
//!
//!     @return number of ASCII parameters
//!
//!
//
//*****************************************************************************


unsigned short itoa(short cNum, char *cString)
{
    char* ptr;
    short uTemp = cNum;
    unsigned short length;

    // value 0 is a special case
    if (cNum == 0)
    {
        length = 1;
        *cString = '0';

        return length;
    }

    // Find out the length of the number, in decimal base
    length = 0;
    while (uTemp > 0)
    {
        uTemp /= 10;
        length++;
    }

    // Do the actual formatting, right to left
    uTemp = cNum;
    ptr = cString + length;
    while (uTemp > 0)
    {
        --ptr;
        *ptr = g_acDigits[uTemp % 10];
        uTemp /= 10;
    }

    return length;
}

//*****************************************************************************
long getHostIP(char* pcHostName, unsigned long * pDestinationIP)
{
    long lStatus = 0;

    lStatus = sl_NetAppDnsGetHostByName((signed char *) pcHostName,
                                            strlen(pcHostName),
                                            pDestinationIP, SL_AF_INET);
//    ASSERT_ON_ERROR(lStatus);
//
    ////System_printf("\nGet Host IP succeeded.\n\rHost: %s IP: %d.%d.%d.%d \n",
     //               pcHostName, SL_IPV4_BYTE(*pDestinationIP,3),
     //               SL_IPV4_BYTE(*pDestinationIP,2),
     //               SL_IPV4_BYTE(*pDestinationIP,1),
     //               SL_IPV4_BYTE(*pDestinationIP,0));
    ////System_flush();

    return lStatus;
}

//*****************************************************************************
//
//! Gets the current time from the selected SNTP server
//!
//! \brief  This function obtains the NTP time from the server.
//!
//! \param  GmtDiffHr is the GMT Time Zone difference in hours
//! \param  GmtDiffMins is the GMT Time Zone difference in minutes
//!
//! \return 0 : success, -ve : failure
//!
//*****************************************************************************
long GetSNTPTime(unsigned char ucGmtDiffHr, unsigned char ucGmtDiffMins)
{

/*
                            NTP Packet Header:

       0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9  0  1
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |LI | VN  |Mode |    Stratum    |     Poll      |   Precision    |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |                          Root  Delay                           |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |                       Root  Dispersion                         |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |                     Reference Identifier                       |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |                                                                |
      |                    Reference Timestamp (64)                    |
      |                                                                |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |                                                                |
      |                    Originate Timestamp (64)                    |
      |                                                                |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |                                                                |
      |                     Receive Timestamp (64)                     |
      |                                                                |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |                                                                |
      |                     Transmit Timestamp (64)                    |
      |                                                                |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |                 Key Identifier (optional) (32)                 |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
      |                                                                |
      |                                                                |
      |                 Message Digest (optional) (128)                |
      |                                                                |
      |                                                                |
      +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

*/
    char cDataBuf[48];
    long lRetVal = 0;
    int iAddrSize;

    // Send a query to the NTP server to get the NTP time
    memset(cDataBuf, 0, sizeof(cDataBuf));
    cDataBuf[0] = '\x1b';

    sAddr.sa_family = AF_INET;
    // the source port
    sAddr.sa_data[0] = 0x00;
    sAddr.sa_data[1] = 0x7B;    // UDP port number for NTP is 123
    sAddr.sa_data[2] = (char)((g_ulDestinationIP>>24)&0xff);
    sAddr.sa_data[3] = (char)((g_ulDestinationIP>>16)&0xff);
    sAddr.sa_data[4] = (char)((g_ulDestinationIP>>8)&0xff);
    sAddr.sa_data[5] = (char)(g_ulDestinationIP&0xff);

    lRetVal = sl_SendTo(g_iSockID, cDataBuf, sizeof(cDataBuf), 0, &sAddr, sizeof(sAddr));
    if (lRetVal != sizeof(cDataBuf))
    {
//    	//System_printf("Could not send request to SNTP server: %i\n",lRetVal);
//    	//System_flush();
        return lRetVal;
    }

    // Wait to receive the NTP time from the server
    sLocalAddr.sin_family = SL_AF_INET;
    sLocalAddr.sin_port = 0;
    sLocalAddr.sin_addr.s_addr = 0;
    if(g_ulElapsedSec == 0)
    {
        lRetVal = sl_Bind(g_iSockID, (SlSockAddr_t *)&sLocalAddr, sizeof(SlSockAddrIn_t));
        if(lRetVal < 0)
        {
  //      	//System_printf("Could not bind to SNTP server: %i\n",lRetVal);
  //      	//System_flush();
            return lRetVal;
        }
    }

    iAddrSize = sizeof(SlSockAddrIn_t);

    lRetVal = sl_RecvFrom(g_iSockID, cDataBuf, sizeof(cDataBuf), 0, (SlSockAddr_t *)&sLocalAddr, (SlSocklen_t*)&iAddrSize);
    if(lRetVal < 0)
    {
  //  	System_printf("Did not receive valid response from SNTP server: %i\n",lRetVal);
  //  	System_flush();
        return lRetVal;
    }

    // Confirm that the MODE is 4 --> server
    if ((cDataBuf[0] & 0x7) != 4)    // expect only server response
    {
        //ASSERT_ON_ERROR(SERVER_GET_TIME_FAILED);  // MODE is not server, abort
    	return -1;
    }
    else
    {
        unsigned char iIndex;

        // Getting the data from the Transmit Timestamp (seconds) field
        // This is the time at which the reply departed the
        // server for the client
        g_ulElapsedSec = cDataBuf[40];
        g_ulElapsedSec <<= 8;
        g_ulElapsedSec += cDataBuf[41];
        g_ulElapsedSec <<= 8;
        g_ulElapsedSec += cDataBuf[42];
        g_ulElapsedSec <<= 8;
        g_ulElapsedSec += cDataBuf[43];

        // seconds are relative to 0h on 1 January 1900
        g_ulElapsedSec -= TIME2013;

        // in order to correct the timezone
        g_ulElapsedSec += (ucGmtDiffHr * SEC_IN_HOUR);
        g_ulElapsedSec += (ucGmtDiffMins * SEC_IN_MIN);

        g_pcCCPtr = &g_acTimeStore[0];

        // day, number of days since beginning of 2013
        g_isGeneralVar = g_ulElapsedSec/SEC_IN_DAY;
        memcpy(g_pcCCPtr, g_acDaysOfWeek2013[g_isGeneralVar%7], 3);
        g_pcCCPtr += 3;
        *g_pcCCPtr++ = '\x20';

        // month
        g_isGeneralVar %= 365;
        for (iIndex = 0; iIndex < 12; iIndex++)
        {
            g_isGeneralVar -= g_acNumOfDaysPerMonth[iIndex];
            if (g_isGeneralVar < 0)
                    break;
        }
        if(iIndex == 12)
        {
            iIndex = 0;
        }
        memcpy(g_pcCCPtr, g_acMonthOfYear[iIndex], 3);
        g_pcCCPtr += 3;
        *g_pcCCPtr++ = '\x20';

        // Set the Month Value
        dateTime.sl_tm_mon = iIndex + 1;

        // date
        // restore the day in current month
        g_isGeneralVar += g_acNumOfDaysPerMonth[iIndex];
        g_uisCCLen = itoa(g_isGeneralVar + 1, g_pcCCPtr);
        g_pcCCPtr += g_uisCCLen;
        *g_pcCCPtr++ = '\x20';

        // Set the Date
        dateTime.sl_tm_day = g_isGeneralVar + 1;

        // time
        g_ulGeneralVar = g_ulElapsedSec%SEC_IN_DAY;

        // number of seconds per hour
        g_ulGeneralVar1 = g_ulGeneralVar%SEC_IN_HOUR;

        // number of hours
        g_ulGeneralVar /= SEC_IN_HOUR;
        g_uisCCLen = itoa(g_ulGeneralVar, g_pcCCPtr);
        g_pcCCPtr += g_uisCCLen;
        *g_pcCCPtr++ = ':';

        // Set the hour
        dateTime.sl_tm_hour = g_ulGeneralVar;

        // number of minutes per hour
        g_ulGeneralVar = g_ulGeneralVar1/SEC_IN_MIN;

        // Set the minutes
        dateTime.sl_tm_min = g_ulGeneralVar;

        // number of seconds per minute
        g_ulGeneralVar1 %= SEC_IN_MIN;
        g_uisCCLen = itoa(g_ulGeneralVar, g_pcCCPtr);
        g_pcCCPtr += g_uisCCLen;
        *g_pcCCPtr++ = ':';
        g_uisCCLen = itoa(g_ulGeneralVar1, g_pcCCPtr);
        g_pcCCPtr += g_uisCCLen;
        *g_pcCCPtr++ = '\x20';

        //Set the seconds
        dateTime.sl_tm_sec = g_ulGeneralVar1;

        // year
        // number of days since beginning of 2013
        g_ulGeneralVar = g_ulElapsedSec/SEC_IN_DAY;
        g_ulGeneralVar /= 365;
        g_uisCCLen = itoa(YEAR2013 + g_ulGeneralVar, g_pcCCPtr);
        g_pcCCPtr += g_uisCCLen;

        *g_pcCCPtr++ = '\0';

        //Set the year
        dateTime.sl_tm_year = 2013 + g_ulGeneralVar;

        //System_printf("Response from server: ");
        //System_printf((char *)g_acSNTPserver);
        //System_printf("\n\r");
        //System_printf(g_acTimeStore);
        //System_printf("\n\r");
        //System_flush();

        //Set time of the device for certificate verification.
        lRetVal = setDeviceTimeDate();
        if(lRetVal < 0)
        {
        	//System_printf("Unable to set time in the device. Error Number: %i\n",lRetVal);
        	//System_flush();
            return lRetVal;
        }
    }

    return SUCCESS;
}

//*****************************************************************************
//
//! This function obtains the current time from a SNTP server if required due
//! to not having current time (when booting up) or periodically to update
//! the time
//!
//! \param None
//!
//! \return  0 on success else error code
//! \return  Error Number of failure
//
//*****************************************************************************
long GetCurrentTime()
{
    int iSocketDesc;
    long lRetVal = -1;

	// Create UDP socket
	iSocketDesc = sl_Socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if(iSocketDesc < 0)
	{
		//System_printf("Could not create UDP socket. Error Number: %i\n",iSocketDesc);
		//System_flush();
		close(iSocketDesc);
		return iSocketDesc;
	}
	g_iSockID = iSocketDesc;

	// Get the NTP server host IP address using the DNS lookup
	lRetVal = getHostIP((char*)g_acSNTPserver, &g_ulDestinationIP);
	if( lRetVal >= 0)
	{
		// Configure the recieve timeout
		struct SlTimeval_t timeVal;
		timeVal.tv_sec =  SERVER_RESPONSE_TIMEOUT;    // Seconds
		timeVal.tv_usec = 0;     // Microseconds. 10000 microseconds resolution
		lRetVal = sl_SetSockOpt(g_iSockID,SL_SOL_SOCKET,SL_SO_RCVTIMEO, (unsigned char*)&timeVal, sizeof(timeVal));
		if(lRetVal < 0)
		{
			//System_printf("Could not configure socket option (receive timeout). Error Number: %i\n",lRetVal);
			//System_flush();
			close(iSocketDesc);
			return lRetVal;
		}
	}
	else
	{
		//System_printf("DNS lookup failed. Error Number: %i\n\r",lRetVal);
		//System_flush();
		close(iSocketDesc);
		return lRetVal;
	}

	// Get current time from the SNTP server
	//System_printf("Fetching Time From SNTP Server\n");
	//System_flush();
	lRetVal = GetSNTPTime(GMT_DIFF_TIME_HRS, GMT_DIFF_TIME_MINS);
	if(lRetVal < 0)
	{
		//System_printf("Server Get Time failed. Error Number: %i\n",lRetVal);
		//System_flush();
		close(iSocketDesc);
		return lRetVal;
	}
	else
	{
		//System_printf("Server Get Time Successful - RTC Updated\n\n");
		//System_flush();
	}

	// Close the socket
	close(iSocketDesc);

	return SUCCESS;
}

