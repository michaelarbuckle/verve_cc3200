/*****************************************************************************
//
// Application Name     - Air Quality Monitor
// Application Overview - This Application captures data from the set of sensors
//							connected to the CC3200 launchpad via I2C bus and UART channle.
//                     1. Connect to CC3200 Launchpad via Mobile App (Or control center TBD)
//                        - Direct Connection to LP by using CC3200 device in
//                          Access Point Mode (Default)
//                        - Add the local SSID and password to WIFI and/or control center
//                     2. by request access to sensor data using Internal HTTP server
//
//    main.c
//    Purpose: Collect environmental data from sensors on I2C and UART Buses
//
//    @author Michael Arbuckle Verve Tech
//    @version 1.1 2/10/17
*/

//****************************************************************************
//
//! \addtogroup oob
//! @{
//
//****************************************************************************

// Standard includes
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// Simplelink includes
#include "simplelink.h"
#include "netcfg.h"

// Driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "utils.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "pin.h"

// OS includes
#include "osi.h"

// Common interface includes
#include "gpio_if.h"
#include "uart_if.h"
#include "i2c_if.h"
#include "common.h"

#include "verve_utils.h"
#include "verve_json.h"
#include "verve_cloud.h"

// App Includes
#include "ads1115.h"
#include "bme280drv.h"
#include "emfdrv.h"
#include "gp2y101drv.h"
#include "mics87drv.h"
#include "radondrv.h"
#include "s8nsairdrv.h"

#include "bma222drv.h"
#include "tmp006drv.h"

// App Includes
#include "smartconfig.h"
#include "pinmux.h"

#define APPLICATION_VERSION              "1.0.0"
#define APP_NAME                         "Air Quality Monitor"
#define OOB_TASK_PRIORITY                1
#define SPAWN_TASK_PRIORITY              9
#define OSI_STACK_SIZE                   2048
#define AP_SSID_LEN_MAX                 32
#define SH_GPIO_3                       3       /* P58 - Device Mode */
#define AUTO_CONNECTION_TIMEOUT_COUNT   50      /* 5 Sec */
#define SL_STOP_TIMEOUT                 200


typedef enum
{
  LED_OFF = 0,
  LED_ON,
  LED_BLINK
}eLEDStatus;

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
static unsigned char POST_token[] = "__SL_P_ULD";

static unsigned int g_uiDeviceModeConfig = ROLE_STA; //default is STA mode
static unsigned char g_ucLEDStatus = LED_OFF;
static unsigned long  g_ulStatus = 0;//SimpleLink Status
static unsigned char  g_ucConnectionSSID[SSID_LEN_MAX+1]; //Connection SSID
static unsigned char  g_ucConnectionBSSID[BSSID_LEN_MAX]; //Connection BSSID
static char g_verveLog[64];





#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


#ifdef USE_FREERTOS
//*****************************************************************************
//
//! Application defined hook (or callback) function - the tick hook.
//! The tick interrupt can optionally call this
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationTickHook( void )
{
}

//*****************************************************************************
//
//! Application defined hook (or callback) function - assert
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vAssertCalled( const char *pcFile, unsigned long ulLine )
{
    while(1)
    {

    }
}

//*****************************************************************************
//
//! Application defined idle task hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationIdleHook( void )
{

}

//*****************************************************************************
//
//! Application provided stack overflow hook function.
//!
//! \param  handle of the offending task
//! \param  name  of the offending task
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationStackOverflowHook( OsiTaskHandle *pxTask, signed char *pcTaskName)
{
    ( void ) pxTask;
    ( void ) pcTaskName;

    for( ;; );
}

void vApplicationMallocFailedHook()
{
    while(1)
  {
    // Infinite loop;
  }
}
#endif

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
//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{

	 UART_PRINT("[WLAN EVENT] event [0x%x]\n\r",
	                       pWlanEvent->Event);

    if(pWlanEvent == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }
    switch(pWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'
            // Applications can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //

            // Copy new connection SSID and BSSID to global parameters
            memcpy(g_ucConnectionSSID,pWlanEvent->EventData.
                   STAandP2PModeWlanConnected.ssid_name,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
            memcpy(g_ucConnectionBSSID,
                   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
                   SL_BSSID_LENGTH);

            UART_PRINT("[WLAN EVENT] Device Connected to the AP: %s , "
                       "BSSID: %x:%x:%x:%x:%x:%x\n\r",
                      g_ucConnectionSSID,g_ucConnectionBSSID[0],
                      g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                      g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                      g_ucConnectionBSSID[5]);
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION
            if(SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("[WLAN EVENT] Device disconnected from the AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on application's "
                           "request \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            else
            {
                UART_PRINT("[WLAN ERROR] Device disconnected from the AP AP: %s, "
                           "BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! \n\r",
                           g_ucConnectionSSID,g_ucConnectionBSSID[0],
                           g_ucConnectionBSSID[1],g_ucConnectionBSSID[2],
                           g_ucConnectionBSSID[3],g_ucConnectionBSSID[4],
                           g_ucConnectionBSSID[5]);
            }
            memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
            memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
        }
        break;

        case SL_WLAN_STA_CONNECTED_EVENT:
        {
            // when device is in AP mode and any client connects to device cc3xxx
            //SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION_FAILED);

            //
            // Information about the connected client (like SSID, MAC etc) will
            // be available in 'slPeerInfoAsyncResponse_t' - Applications
            // can use it if required
            //
            // slPeerInfoAsyncResponse_t *pEventData = NULL;
            // pEventData = &pSlWlanEvent->EventData.APModeStaConnected;
            //

            UART_PRINT("[WLAN EVENT] Station connected to device\n\r");
        }
        break;

        case SL_WLAN_STA_DISCONNECTED_EVENT:
        {
            // when client disconnects from device (AP)
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            //
            // Information about the connected client (like SSID, MAC etc) will
            // be available in 'slPeerInfoAsyncResponse_t' - Applications
            // can use it if required
            //
            // slPeerInfoAsyncResponse_t *pEventData = NULL;
            // pEventData = &pSlWlanEvent->EventData.APModestaDisconnected;
            //
            UART_PRINT("[WLAN EVENT] Station disconnected from device\n\r");
        }
        break;

        case SL_WLAN_SMART_CONFIG_COMPLETE_EVENT:
        {
            //SET_STATUS_BIT(g_ulStatus, STATUS_BIT_SMARTCONFIG_START);

            //
            // Information about the SmartConfig details (like Status, SSID,
            // Token etc) will be available in 'slSmartConfigStartAsyncResponse_t'
            // - Applications can use it if required
            //
            //  slSmartConfigStartAsyncResponse_t *pEventData = NULL;
            //  pEventData = &pSlWlanEvent->EventData.smartConfigStartResponse;
            //

        }
        break;

        case SL_WLAN_SMART_CONFIG_STOP_EVENT:
        {
            // SmartConfig operation finished
            //CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_SMARTCONFIG_START);

            //
            // Information about the SmartConfig details (like Status, padding
            // etc) will be available in 'slSmartConfigStopAsyncResponse_t' -
            // Applications can use it if required
            //
            // slSmartConfigStopAsyncResponse_t *pEventData = NULL;
            // pEventData = &pSlWlanEvent->EventData.smartConfigStopResponse;
            //
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event [0x%x]\n\r",
                       pWlanEvent->Event);
        }
        break;
    }
}

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(pNetAppEvent == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }

    switch(pNetAppEvent->Event)
    {
        case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
        {
            SlIpV4AcquiredAsync_t *pEventData = NULL;

            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            //Ip Acquired Event Data
            pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

            UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                       "Gateway=%d.%d.%d.%d\n\r",
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
            SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0));

            UNUSED(pEventData);
        }
        break;

        case SL_NETAPP_IP_LEASED_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            //
            // Information about the IP-Leased details(like IP-Leased,lease-time,
            // mac etc) will be available in 'SlIpLeasedAsync_t' - Applications
            // can use it if required
            //
            // SlIpLeasedAsync_t *pEventData = NULL;
            // pEventData = &pNetAppEvent->EventData.ipLeased;
            //

        }
        break;

        case SL_NETAPP_IP_RELEASED_EVENT:
        {
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            //
            // Information about the IP-Released details (like IP-address, mac
            // etc) will be available in 'SlIpReleasedAsync_t' - Applications
            // can use it if required
            //
            // SlIpReleasedAsync_t *pEventData = NULL;
            // pEventData = &pNetAppEvent->EventData.ipReleased;
            //
        }
		break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}

static long ConnectToAP()
{
    long lRetVal = -1;

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its desired state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();
    if(lRetVal < 0)
    {
        if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
        {
            UART_PRINT("Failed to configure the device in its default state, "
                            "Error-code: %d\n\r", DEVICE_NOT_IN_STATION_MODE);
        }

        return -1;
    }

    UART_PRINT("Device is configured in default state \n\r");

    //
    // Assumption is that the device is configured in station mode already
    // and it is in its default state
    //
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0 || ROLE_STA != lRetVal)
    {
        ASSERT_ON_ERROR(DEVICE_START_FAILED);
    }

    UART_PRINT("Device started as STATION \n\r");

    // Connecting to WLAN AP - Set with static parameters defined at the top
    // After this call we will be connected and have IP address
    lRetVal = WlanConnect();

  //  UART_PRINT("Connected to the AP: %s\r\n", SSID_NAME);
    return 0;
}



//*****************************************************************************
//                 GLOBAL VARIABLES -- End


int getTemperatureElements(msg_format msg_type,char* dest )
{
            float fCurrentTemp;
            TMP006DrvGetTemp(&fCurrentTemp);
        	char value[8];
        	char element[48];
        	sprintf(value,"%0.2f",fCurrentTemp);
        	sprintf(element,(char *)SENSORS_TEMPLATE_B,Name_TEMP,value,(char *)SENSORS_SEPARATOR);
        	strncat(dest, element, strlen(element));
}
int getMotionElements(msg_format msg_type,char* dest )
{
	char element[32];
	char value[2];

	int ucMotion = 0;
    //Define Accelerometer Threshold to Detect Movement
    const short csAccThreshold    = 5;

    signed char cAccXT1,cAccYT1,cAccZT1;
    signed char cAccXT2,cAccYT2,cAccZT2;
    signed short sDelAccX, sDelAccY, sDelAccZ;
    int iRet = -1;
    int iCount = 0;

    iRet = BMA222ReadNew(&cAccXT1, &cAccYT1, &cAccZT1);
    if(iRet)
    {
        //In case of error/ No New Data return
        return 1;
    }
    for(iCount=0;iCount<2;iCount++)
    {
        MAP_UtilsDelay((90*80*1000)); //30msec
        iRet = BMA222ReadNew(&cAccXT2, &cAccYT2, &cAccZT2);
        if(iRet)
        {
            //In case of error/ No New Data continue
            iRet = 0;
            continue;
        }

        else
        {
            sDelAccX = abs((signed short)cAccXT2 - (signed short)cAccXT1);
            sDelAccY = abs((signed short)cAccYT2 - (signed short)cAccYT1);
            sDelAccZ = abs((signed short)cAccZT2 - (signed short)cAccZT1);

            //Compare with Pre defined Threshold
            if(sDelAccX > csAccThreshold || sDelAccY > csAccThreshold ||
               sDelAccZ > csAccThreshold)
            {
                //Device Movement Detected, Break and Return
                ucMotion = 1;
                break;
            }
            else
            {
                //Device Movement Static
            	ucMotion = 0;
            }
        }
    }
	sprintf(value,"%d",ucMotion);

	sprintf(element,SENSORS_TEMPLATE_B,Name_ACC,value,SENSORS_SEPARATOR);
	strncat(dest, element, strlen(element));

	return 1;
}


int getAHPTElements(msg_format msg_type,char* dest )
{
	int tt;

	char element[32];

	char value[8];

g_BME280Data.bActive = 0;
g_BME280Data.fHumidity = -1.0;
g_BME280Data.fPressure = -1.0;
g_BME280Data.fAltitude = -1.0;
g_BME280Data.fTemperature = -1.0;

	tt = BME280Read(&g_BME280Data.bActive, &g_BME280Data.fHumidity, &g_BME280Data.fPressure, &g_BME280Data.fAltitude, &g_BME280Data.fTemperature);
	sprintf(value,"%0.2f",g_BME280Data.fHumidity);

	sprintf(element,(char *)SENSORS_TEMPLATE_B,Name_Humidity,value,(char *)SENSORS_SEPARATOR);
	strncat(dest, element, strlen(element));

	sprintf(value,"%f",g_BME280Data.fPressure);
	sprintf(value,SENSORS_TEMPLATE_B,Name_Pressure,value,SENSORS_SEPARATOR);
	strncat(dest, element, strlen(element));

	sprintf(value,"%0.2f",g_BME280Data.fTemperature);
	sprintf(element,(char *)SENSORS_TEMPLATE_B,Name_TEMP,value,SENSORS_SEPARATOR);
	strncat(dest, element, strlen(element));

	return tt;
}

int getCO2Elements(msg_format msg_type,char* dest )
{
	int cVal;
	char value[8];
	char element[32];
	char logs[64];

	g_S8NSAIRData.bActive = 0;
	g_S8NSAIRData.fCO2 = -1;

	cVal = S8NSAIRDrvGetCO2(&g_S8NSAIRData.bActive,&g_S8NSAIRData.fCO2, logs);
	sprintf(value,"%f",g_S8NSAIRData.fCO2);

	sprintf(element,SENSORS_TEMPLATE_B,Name_CO2,value,SENSORS_SEPARATOR);
	strncat(dest, value, strlen(value));
    return cVal;
}



int getDustElements(msg_format msg_type,char* dest )
{
	g_GP2Y101Data.bActive=0;
	g_GP2Y101Data.fDust =-1.0;
	int s=GP2Y101Read(&g_GP2Y101Data.bActive,&g_GP2Y101Data.fDust);

	char value[8];
	char element[32];
	sprintf(value,"%.3f",g_GP2Y101Data.fDust);
	sprintf(element,SENSORS_TEMPLATE_B,Name_Dust,value,SENSORS_SEPARATOR);
	strncat(dest, element, strlen(element));

}

int getEMFElements(msg_format msg_type,char* dest )
{


	 g_EMFData.bActive = -1;
	 g_EMFData.fIntensity = -1.0;

	int s=EMFRead(&g_EMFData.bActive, &g_EMFData.fIntensity);
	char value[8];
	char element[32];
	sprintf(value,"%.3f",g_EMFData.fIntensity);
	sprintf(element,SENSORS_TEMPLATE_B,Name_EMF,value,SENSORS_SEPARATOR);
	strncat(dest, element, strlen(element));
}

int getNOxElements(msg_format msg_type,char* dest )
{
	char value[8];
	char element[32];
	value[0]='0';
	value[1]='.';
	value[2]='0';
	value[3]='0';
	value[4]='\0';
	//sprintf(value,"%.3f",g_EMFData.fIntensity);
	sprintf(element,SENSORS_TEMPLATE_B,Name_NOX,value,SENSORS_SEPARATOR);
	strncat(dest, element, strlen(element));
   }


int createRadonElements(msg_format msg_type,char* dest )
{

	char logs[64];
	g_RadonData.bActive = -1;
	g_RadonData.fRadon =-1.0f;
	int stat= radonDrvGetRadon(&g_RadonData.bActive,&g_RadonData.fRadon,logs);
	char value[8];
	char element[32];
	sprintf(value,"%.3f",g_RadonData.fRadon);
	sprintf(element,SENSORS_TEMPLATE_B,Name_Radon,value,SENSORS_SEPARATOR);
	strncat(dest, element, strlen(element));

    }
int createVOCElements(msg_format msg_type,char* dest )
{
	char err[32];
	g_MICS87Data.bActive = -1;
	g_MICS87Data.fVOC=-1;
	int stat = MICS87Read(&g_MICS87Data.bActive,&g_MICS87Data.fVOC,err);
	char value[8];
	char element[32];
	sprintf(value,"%f",g_MICS87Data.fVOC);
	sprintf(element,SENSORS_TEMPLATE_B,Name_VOC,value,SENSORS_SEPARATOR);
	strncat(dest, element, strlen(element));

}

int createSensorsMsg(msg_format msg_type, char* msg,char* uuid, int maxSz)
 {
	int retVal=0;
	char element[64];
	char u[] =  "test.UUID.0001";//getMACAddressForDevice();;
	strcpy(uuid,u);
	const char* dt = "20170408 17:27.32";//getDateTimeString();
	sprintf(element,SENSORS_TEMPLATE_A,uuid,dt);
	strncat(msg, element, strlen(element));
	retVal = getAHPTElements(msg_type,msg);
	retVal = getDustElements(msg_type,msg);
	retVal = getEMFElements(msg_type,msg);
	retVal = getNOxElements(msg_type,msg);
	retVal = createRadonElements(msg_type,msg);
	retVal = createVOCElements(msg_type,msg );

	char c[]="]}";
	strcat(msg, c);

 }

int createOOBSensorsMsg(msg_format msg_type, char* msg,char* uuid, int maxSz)
 {
	int retVal=0;
	char element[64];
	char u[] =  "test.UUID.0001";//getMACAddressForDevice();;
	strcpy(uuid,u);
	const char* dt = "20170408 17:27.32";//getDateTimeString();
	sprintf(element,SENSORS_TEMPLATE_A,uuid,dt);
	strncat(msg, element, strlen(element));
	retVal = getMotionElements(msg_type,msg );
	retVal = getTemperatureElements( msg_type,msg);

	char c[]="]}";
	strcat(msg, c);

 }


//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pSlHttpServerEvent, 
                                SlHttpServerResponse_t *pSlHttpServerResponse)
{
    switch (pSlHttpServerEvent->Event)
    {
        case SL_NETAPP_HTTPGETTOKENVALUE_EVENT:
        {
            unsigned char *ptr;

            ptr = pSlHttpServerResponse->ResponseData.token_value.data;
            pSlHttpServerResponse->ResponseData.token_value.len = 0;

			if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
					GET_token_AHPT, strlen((const char *)GET_token_AHPT)) == 0)
			            {

				g_BME280Data.bActive = 0;
				g_BME280Data.fHumidity = -1.0;
				g_BME280Data.fPressure = -1.0;
				g_BME280Data.fAltitude = -1.0;
				g_BME280Data.fTemperature = -1.0;

				int tt=BME280Read(&g_BME280Data.bActive, &g_BME280Data.fHumidity, &g_BME280Data.fPressure, &g_BME280Data.fAltitude, &g_BME280Data.fTemperature);
				char* value[64];
				sprintf(value,"%.2f:%.2f:%.0f:%.2f",g_BME280Data.fAltitude,g_BME280Data.fHumidity,g_BME280Data.fPressure,g_BME280Data.fTemperature);


				strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,value);
                pSlHttpServerResponse->ResponseData.token_value.len += strlen(value);
			   }

			if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
					GET_token_Cloud, strlen((const char *)GET_token_Cloud)) == 0)
					            {
			char msg[256];
			char uuid[32];
			int rVal =createSensorsMsg(WEB_SVC, msg, uuid, 256);
			rVal =HTTPWriteToCloud(CLOUD_HOST_NAME ,msg, uuid);

					            }
			if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
					GET_token_Control, strlen((const char *)GET_token_Control)) == 0)
					            {
				char msg[256];
				char uuid[32];
				int rVal = createSensorsMsg(WEB_SVC, msg, uuid, 256);
				rVal =HTTPWriteToCloud(CONTROL_HOST_NAME ,msg, uuid);

					            }

			if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
					GET_token_CO2, strlen((const char *)GET_token_CO2)) == 0)
			            {
//				int init= S8NSAIRUARTInitRx();


				int cVal;
				char value[64];
				char logs[55];

				g_S8NSAIRData.bActive = 0;
				g_S8NSAIRData.fCO2 = -1;

				cVal = S8NSAIRDrvGetCO2(&g_S8NSAIRData.bActive,&g_S8NSAIRData.fCO2, logs);

//				sprintf(value,"%.1f:%s",g_S8NSAIRData.fCO2,logs);
				sprintf(value,"%.1f",g_S8NSAIRData.fCO2);
				strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,(char*)value);
                pSlHttpServerResponse->ResponseData.token_value.len += strlen(value);

			}


			if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
					GET_token_CO2_Calibrate, strlen((const char *)GET_token_CO2_Calibrate)) == 0)
			{
				int cVal;
				char logs[64];
				cVal = S8NSAIRDrvCalibrate(logs);

				strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,(char*)logs);
                pSlHttpServerResponse->ResponseData.token_value.len += strlen(logs);

			            }

			if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
					GET_token_DateTime, strlen((const char *)GET_token_DateTime)) == 0)
			            {
				const char* dt = "20170428 19:27.32";//getDateTimeString();
                strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,dt);
                pSlHttpServerResponse->ResponseData.token_value.len += strlen(dt);

			            }


			if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
					GET_token_Dust, strlen((const char *)GET_token_Dust)) == 0)
			            {

				g_GP2Y101Data.bActive=0;
				g_GP2Y101Data.fDust =-1.0;
				g_GP2Y101Data.raw = 0;
				g_GP2Y101Data.normalized = 0.0;
				int s=GP2Y101Read(&g_GP2Y101Data.bActive,&g_GP2Y101Data.fDust);

				char value[32];
				//sprintf(value,"'%d:%.3f:%.4f'",g_GP2Y101Data.raw,g_GP2Y101Data.normalized,g_GP2Y101Data.fDust);
				sprintf(value,"%.3f",g_GP2Y101Data.fDust);

                strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,value);
                pSlHttpServerResponse->ResponseData.token_value.len += strlen(value);
			            }

			if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
					GET_token_EMF, strlen((const char *)GET_token_EMF)) == 0)
			            {

				 g_EMFData.bActive = -1;
				 g_EMFData.fIntensity = -1.0;
				 g_EMFData.raw = 0;
				 g_EMFData.normalized = 0.0;

				int s=EMFRead(&g_EMFData.bActive, &g_EMFData.fIntensity);
				char value[32];
//				sprintf(value,"%d:%.2f:%.2f",g_EMFData.raw,g_EMFData.normalized,g_EMFData.fIntensity);
				sprintf(value,"%.2f",g_EMFData.fIntensity);
                strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,value);
                pSlHttpServerResponse->ResponseData.token_value.len += strlen(value);
			            }

			if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
					GET_token_Humidity, strlen((const char *)GET_token_Humidity)) == 0)
			            {

				g_BME280Data.bActive = 0;
				g_BME280Data.fHumidity = -1.0;
				g_BME280Data.fPressure = -1.0;
				g_BME280Data.fAltitude = -1.0;
				g_BME280Data.fTemperature = -1.0;

				int tt=BME280Read(&g_BME280Data.bActive, &g_BME280Data.fHumidity, &g_BME280Data.fPressure, &g_BME280Data.fAltitude, &g_BME280Data.fTemperature);
				char* value[64];
				sprintf(value,"%.2f",g_BME280Data.fHumidity);


				strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,value);
                pSlHttpServerResponse->ResponseData.token_value.len += strlen(value);
			   }

		    if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
		                    GET_token_ACC, strlen((const char *)GET_token_ACC)) == 0)
		            {

		    	       int motion = ReadAccSensor();
						char value[8];
						sprintf(value,"%d",motion);

						strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,value);
		                pSlHttpServerResponse->ResponseData.token_value.len += strlen(value);

		            }

			if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
					GET_token_NOX, strlen((const char *)GET_token_NOX)) == 0)
			            {
				char value[64];
				value[0]='0';
				value[1]='.';
				value[2]='0';
				value[3]='0';
				value[4]='\0';
				strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,value);
                pSlHttpServerResponse->ResponseData.token_value.len += strlen(value);
			   }

			if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
					GET_token_Pressure, strlen((const char *)GET_token_Pressure)) == 0)
			{

				g_BME280Data.bActive = 0;
				g_BME280Data.fHumidity = -1.0;
				g_BME280Data.fPressure = -1.0;
				g_BME280Data.fAltitude = -1.0;
				g_BME280Data.fTemperature = -1.0;

				int tt=BME280Read(&g_BME280Data.bActive, &g_BME280Data.fHumidity, &g_BME280Data.fPressure, &g_BME280Data.fAltitude, &g_BME280Data.fTemperature);
				char* value[64];
				sprintf(value,"%.0f",g_BME280Data.fPressure);


				strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,value);
                pSlHttpServerResponse->ResponseData.token_value.len += strlen(value);
			   }

			if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
								GET_token_RadonLogs, strlen((const char *)GET_token_RadonLogs)) == 0)
						            {

							char logs[64];
							g_RadonData.bActive = -1;
							g_RadonData.fRadon =-1.0f;
							int stat= radonDrvGetRadon(&g_RadonData.bActive,&g_RadonData.fRadon,logs);

					        strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,(char*)logs);
			                pSlHttpServerResponse->ResponseData.token_value.len += strlen(logs);

						    }


			if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
					GET_token_Radon, strlen((const char *)GET_token_Radon)) == 0)
			            {

				char logs[64];
				char value[8];
				g_RadonData.bActive = -1;
				g_RadonData.fRadon = 0.0f;
				//int stat= radonDrvGetRadon(&g_RadonData.bActive,&g_RadonData.fRadon,logs);

				sprintf(value,"%.0f",g_RadonData.fRadon);

		        strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,(char*)value);
                pSlHttpServerResponse->ResponseData.token_value.len += strlen(value);

			    }

            if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
                    GET_token_TEMP, strlen((const char *)GET_token_TEMP)) == 0)
            {
                float fCurrentTemp;
                TMP006DrvGetTemp(&fCurrentTemp);
                char cTemp = (char)fCurrentTemp;
                short sTempLen = itoa(cTemp,(char*)ptr);
                ptr[sTempLen++] = ' ';
                ptr[sTempLen] = 'F';
                pSlHttpServerResponse->ResponseData.token_value.len += sTempLen;

            }



			if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
					GET_token_UUID, strlen((const char *)GET_token_UUID)) == 0)
			            {


			//	char* uuid =  getMACAddressForDevice();//"test.UUID.0001";//
/*				int i;
				int lReport;

				unsigned char macAddressVal[SL_MAC_ADDR_LEN];
				unsigned char macAddressLen = SL_MAC_ADDR_LEN;

				memset(macAddressVal, '\0', sizeof(macAddressVal));
				lReport = sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,macAddressVal);

				int ii,dCount;
				int vLen = 0;
				unsigned char rBuf[12];
				char value[32];
				for (i = 0 ; i < SL_MAC_ADDR_LEN ; i++)
				{

				sprintf(rBuf,"%x",macAddressVal[i]);
				dCount = strlen(rBuf);
				for ( ii=0;ii < dCount;ii++)		 //	 logs[logLen++]=(char)wChar;S
				{
					value[vLen++]=rBuf[ii];
				}
				}
				value[vLen]='\0';

				/*
				sprintf(value, "%s",(char *) macAddressVal);

				char macAddressPart[2];
				char macAddressFull[18]; //18
				memset(macAddressFull, '\0', sizeof(macAddressFull));

				for (i = 0 ; i < 6 ; i++)
				{
					sprintf(macAddressPart, "%02X", macAddressVal[i]);
					strcat(macAddressFull, (char *)macAddressPart);
					strcat(macAddressFull, ":");
				}
			*/
					char* value =  "test.UUID.0001";//getMACAddressForDevice();//

                strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,value);
                pSlHttpServerResponse->ResponseData.token_value.len += strlen(value);

			            }



            if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
            		GET_token_VOC, strlen((const char *)GET_token_VOC)) == 0)
                        {

            	char err[32];
            	g_MICS87Data.bActive = -1;
            	g_MICS87Data.fVOC=321.23;
            	int stat = MICS87Read(&g_MICS87Data.bActive,&g_MICS87Data.fVOC,err);

				char value[64];
//				sprintf(value,"%d:%.2f:%.2f",g_MICS87Data.raw,g_MICS87Data.normalized,g_MICS87Data.fVOC);
				sprintf(value,"%.2f",g_MICS87Data.fVOC);
                strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,value);
                pSlHttpServerResponse->ResponseData.token_value.len += strlen(value);

                        }

			if(memcmp(pSlHttpServerEvent->EventData.httpTokenName.data,
					GET_token_Logger, strlen((const char *)GET_token_Logger)) == 0)
			            {

				UARTClear();
				//Add to log
				short ll = strlen(g_verveLog);
				if (ll == 0)
				{
					g_verveLog[0] ='e';
					g_verveLog[1] ='m';
					g_verveLog[2] ='p';
					g_verveLog[3] ='t';
					g_verveLog[4] ='y';
					g_verveLog[5] ='\0';
				} else if (ll >=64)
				{
					g_verveLog[0] ='o';
					g_verveLog[1] ='v';
					g_verveLog[2] ='e';
					g_verveLog[3] ='r';
					g_verveLog[4] ='f';
					g_verveLog[5] ='\0';
				}
				char logs[24];
				checkCRC(logs, 0);

                strcpy((char*)pSlHttpServerResponse->ResponseData.token_value.data,logs);
                pSlHttpServerResponse->ResponseData.token_value.len += strlen(logs);

			            }


            break;
        }

        case SL_NETAPP_HTTPPOSTTOKENVALUE_EVENT:
        {

        	int cnt = 0;
            unsigned char led;
            unsigned char *ptr = pSlHttpServerEvent->EventData.httpPostData.token_name.data;
            unsigned char pcSsidName[33];


            //g_ucLEDStatus = 0;
            if(memcmp(ptr, POST_token, strlen((const char *)POST_token)) == 0)
            {
            //	pSlHttpServerEvent->EventData.httpPostData.action
                ptr = pSlHttpServerEvent->EventData.httpPostData.token_value.data;

                if(memcmp(ptr, "AP ON", 5) != 0)
                {



                } else   if(memcmp(ptr, "SSID", 4) != 0)
                {
                    ptr += 4;
                    while ((pcSsidName[cnt++]=*ptr ) != '\0'&& cnt < 33)
                    {
                    	ptr += 1;
                	}

            //       int retVal = ConfigureMode(ROLE_AP, pcSsidName, cnt);

                }
                else  if(memcmp(ptr, "LED", 3) == 0)
                 //   break;
                {
                ptr += 3;
                led = *ptr;
                ptr += 2;
                if(led == '1')
                {
                    if(memcmp(ptr, "ON", 2) == 0)
                    {
                        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                                                g_ucLEDStatus = LED_ON;

                    }
                    else if(memcmp(ptr, "Blink", 5) == 0)
                    {
                        //GPIO_IF_LedOn(MCU_RED_LED_GPIO);
                        g_ucLEDStatus = LED_BLINK;
                    }
                    else
                    {
                        //GPIO_IF_LedOff(MCU_RED_LED_GPIO);
                                                g_ucLEDStatus = LED_OFF;
                    }
                }
                else if(led == '2')
                {
                    if(memcmp(ptr, "ON", 2) == 0)
                    {
                        //GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                    }
                    else if(memcmp(ptr, "Blink", 5) == 0)
                    {
                        //GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
                        g_ucLEDStatus = 1;
                    }
                    else
                    {
                        //GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
                    }
                }

            }
          }//if begins with ==
        }
            break;
        default:
            break;
    }
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if(pDevEvent == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    if(pSock == NULL)
    {
        UART_PRINT("Null pointer\n\r");
        LOOP_FOREVER();
    }
    //
    // This application doesn't work w/ socket - Events are not expected
    //
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status)
            {
                case SL_ECLOSE: 
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n", 
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default: 
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
        	UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }
}


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************

//*****************************************************************************
//
//! \brief This function initializes the application variables
//!
//! \param    None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeAppVariables()
{
    g_ulStatus = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    g_uiDeviceModeConfig = ROLE_STA; //default is STA mode
    g_ucLEDStatus = LED_OFF;    
    g_verveLog[0]='\0';
    g_ucMotion = -1;

}


//****************************************************************************
//
//! Confgiures the mode in which the device will work
//!
//! \param iMode is the current mode of the device
//!
//!
//! \return   SlWlanMode_t
//!                        
//
//****************************************************************************
static int ConfigureMode(int iMode)
{

	int setApDomainName();

	long   lRetVal = -1;

    lRetVal = sl_WlanSetMode(iMode);
    ASSERT_ON_ERROR(lRetVal);

    /* Restart Network processor */
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    // reset status bits
    CLR_STATUS_BIT_ALL(g_ulStatus);

    return sl_Start(NULL,NULL,NULL);
}


//****************************************************************************
//
//!    \brief Connects to the Network in AP or STA Mode - If ForceAP Jumper is
//!                                             Placed, Force it to AP mode
//!
//! \return  0 - Success
//!            -1 - Failure
//
//****************************************************************************
long ConnectToNetwork()
{
    long lRetVal = -1;
    unsigned int uiConnectTimeoutCnt =0;

    // staring simplelink
    lRetVal =  sl_Start(NULL,NULL,NULL);
    ASSERT_ON_ERROR( lRetVal);
    UART_PRINT("[SL Start [%ld]\n\r",
                          lRetVal);

    // Device is in AP Mode and Force AP Jumper is not Connected
    if(ROLE_STA != lRetVal && g_uiDeviceModeConfig == ROLE_STA )
    {
        if (ROLE_AP == lRetVal)
        {
            // If the device is in AP mode, we need to wait for this event 
            // before doing anything 
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
            #ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask(); 
            #endif
            }
        }
        //Switch to STA Mode
        lRetVal = ConfigureMode(ROLE_STA);
        ASSERT_ON_ERROR( lRetVal);
    }



    //Device is in STA Mode and Force AP Jumper is Connected
    if(ROLE_AP != lRetVal && g_uiDeviceModeConfig == ROLE_AP )
    {
         //Switch to AP Mode
         lRetVal = ConfigureMode(ROLE_AP);
         ASSERT_ON_ERROR( lRetVal);

    }

    //No Mode Change Required
    if(lRetVal == ROLE_AP)
    {
        //waiting for the AP to acquire IP address from Internal DHCP Server
        // If the device is in AP mode, we need to wait for this event 
        // before doing anything 
        while(!IS_IP_ACQUIRED(g_ulStatus))
        {
        #ifndef SL_PLATFORM_MULTI_THREADED
            _SlNonOsMainLoopTask(); 
        #endif
        }
        //Stop Internal HTTP Server
        lRetVal = sl_NetAppStop(SL_NET_APP_HTTP_SERVER_ID);
        ASSERT_ON_ERROR( lRetVal);

        //Start Internal HTTP Server
        lRetVal = sl_NetAppStart(SL_NET_APP_HTTP_SERVER_ID);
        ASSERT_ON_ERROR( lRetVal);

       char cCount=0;
       
       //Blink LED 3 times to Indicate AP Mode
       for(cCount=0;cCount<3;cCount++)
       {
           //Turn RED LED On
           GPIO_IF_LedOn(MCU_RED_LED_GPIO);
           osi_Sleep(400);
           
           //Turn RED LED Off
           GPIO_IF_LedOff(MCU_RED_LED_GPIO);
           osi_Sleep(400);
       }

       char ssid[32];
	   unsigned short len = 32;
	   unsigned short config_opt = WLAN_AP_OPT_SSID;
	   sl_WlanGet(SL_WLAN_CFG_AP_ID, &config_opt , &len, (unsigned char* )ssid);
	   UART_PRINT("\n\r Connect to : \'%s\'\n\r\n\r",ssid);
    }
    else
    {

        UART_PRINT("more station mode");

    	//Stop Internal HTTP Server
        lRetVal = sl_NetAppStop(SL_NET_APP_HTTP_SERVER_ID);
        ASSERT_ON_ERROR( lRetVal);

        //Start Internal HTTP Server
        lRetVal = sl_NetAppStart(SL_NET_APP_HTTP_SERVER_ID);
        ASSERT_ON_ERROR( lRetVal);

    	//waiting for the device to Auto Connect
        while(uiConnectTimeoutCnt<AUTO_CONNECTION_TIMEOUT_COUNT &&
            ((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))) 
        {
            //Turn RED LED On
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            osi_Sleep(50);
            
            //Turn RED LED Off
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            osi_Sleep(50);
            
            uiConnectTimeoutCnt++;
        }
        //Couldn't connect Using Auto Profile
        if(uiConnectTimeoutCnt == AUTO_CONNECTION_TIMEOUT_COUNT)
        {
            //Blink Red LED to Indicate Connection Error
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            
            CLR_STATUS_BIT_ALL(g_ulStatus);

            UART_PRINT("[connect with smart config");


            //Connect Using Smart Config
            lRetVal = SmartConfigConnect();
            ASSERT_ON_ERROR(lRetVal);

            //Waiting for the device to Auto Connect
            while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
            {
                MAP_UtilsDelay(500);              
                UART_PRINT("waiting to connect with smart config");

            }
    }
    //Turn RED LED Off
    GPIO_IF_LedOff(MCU_RED_LED_GPIO);


    }
    return SUCCESS;
}


//****************************************************************************
//
//!    \brief Read Force AP GPIO and Configure Mode - 1(Access Point Mode)
//!                                                  - 0 (Station Mode)
//!
//! \return                        None
//
//****************************************************************************
static void ReadDeviceConfiguration()
{
    unsigned int uiGPIOPort;
    unsigned char pucGPIOPin;
    unsigned char ucPinValue;
        
    //Read GPIO
    GPIO_IF_GetPortNPin(SH_GPIO_3,&uiGPIOPort,&pucGPIOPin);
    ucPinValue = GPIO_IF_Get(SH_GPIO_3,uiGPIOPort,pucGPIOPin);
        
    //If Connected to VCC, Mode is AP
    if(ucPinValue == 1)
    {
        //AP Mode
        g_uiDeviceModeConfig = ROLE_AP;
        	UART_PRINT("AP MODE \n\r");
    }
    else
    {
        //STA Mode
        g_uiDeviceModeConfig = ROLE_STA;
    	UART_PRINT("Read device STATION MODE \n\r");

    }

}

//****************************************************************************
//
//!    \brief OOB Application Main Task - Initializes SimpleLink Driver and
//!                                              Handles HTTP Requests
//! \param[in]                  pvParameters is the data passed to the Task
//!
//! \return                        None
//
//****************************************************************************
static void MonitorTask(void *pvParameters)
{
    long   lRetVal = -1;

    //Read Device Mode Configuration
    ReadDeviceConfiguration();

    //Connect to Network
    lRetVal = ConnectToNetwork();
    if(lRetVal < 0)
    {
    	UART_PRINT("NOT CONNECTED TO NETWORK");
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }
    UART_PRINT("connected");


    //Handle Async Events
    while(1)

    {
        //LED Actions
        if(g_ucLEDStatus == LED_ON)
        {
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            osi_Sleep(500);
        }
        if(g_ucLEDStatus == LED_OFF)
        {
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            osi_Sleep(500);
        }
        if(g_ucLEDStatus==LED_BLINK)
        {
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
            osi_Sleep(500);
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
            osi_Sleep(500);
        }


       // osi_Sleep(900000);//sleep for 15 minuutes chage to using timer with interupt
       // hibernateWhenNoActvityfor a while

        UART_PRINT("abOUT TO WRITE WEB SRVC \n\r");

        //save sample to Control Center
		//int rVal = createSensorsMsg(WEB_SVC, msg, uuid, 256);
		//char msg[256];
		char msg[]="{\"uuid\":\"1234-verve-json-oob-test\",\"timestamp\":\"20170501 00:00.01\",\"sensors\":[ {\"type\":\"CO2\",\"value\":\"444\"}]}";
		//char uuid[32];
		char uuid[]="1234-verve-json-oob-test";

		int rVal = HTTPWriteToCloud(CONTROL_HOST_NAME ,msg, uuid);

		UART_PRINT("wrote to web srvc");
		osi_Sleep(10000);

    }
}


//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif  //ccs
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif  //ewarm
    
#endif  //USE_TIRTOS
    
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

void
InitUARTBus()
{
	S8NSAIRUARTInit();

}
static void
   DisplayBanner(char * AppName)
   {

       UART_PRINT("\n\n\n\r");
       UART_PRINT("\t\t *************************************************\n\r");
       UART_PRINT("\t\t           CC3200 %s Application       \n\r", AppName);
       UART_PRINT("\t\t *************************************************\n\r");
       UART_PRINT("\n\n\n\r");
   }

//****************************************************************************
//                            MAIN FUNCTION
//****************************************************************************
void main()
{
    long   lRetVal = -1;

    //
    // Board Initilization
    //
    BoardInit();
    
    //
    // Configure the pinmux settings for the peripherals exercised
    //
    PinMuxConfig();

    PinConfigSet(PIN_58,PIN_STRENGTH_2MA|PIN_STRENGTH_4MA,PIN_TYPE_STD_PD);

    // Initialize Global Variables
    InitializeAppVariables();
    
    //
    // LED Init
    //
    GPIO_IF_LedConfigure(LED1);

    //Turn Off the LEDs
    GPIO_IF_LedOff(MCU_RED_LED_GPIO);

    InitTerm();

    DisplayBanner(APP_NAME);

    //
    // LED Init
    //
    GPIO_IF_LedConfigure(LED1);
      
    //Turn Off the LEDs
    GPIO_IF_LedOff(MCU_RED_LED_GPIO);
       
    //
    // I2C Init
    //
    lRetVal = I2C_IF_Open(I2C_MASTER_MODE_FST);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }    

    //Init Temprature Sensor
    lRetVal = TMP006DrvOpen();
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }    

    //Init Accelerometer Sensor
    lRetVal = BMA222Open();
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }    

    //
    // Simplelinkspawntask
    //
    lRetVal = VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }    

    
    //
    // Create monitor Task
    //
    lRetVal = osi_TaskCreate(MonitorTask, (signed char*)"MonitorTask", \
                                OSI_STACK_SIZE, NULL, \
                                OOB_TASK_PRIORITY, NULL );
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }    



    //
    // Start OS Scheduler
    //
    osi_start();


    //let the startup happen

    while (1)
    {/*
    	char msg[256];
    	char uuid[32];
    	int rVal = createOOBSensorsMsg(WEB_SVC, msg, uuid, 256);
    	rVal =HTTPWriteToCloud(CONTROL_HOST_NAME ,msg, uuid);
    	osi_Sleep(100000);
    */
    }

}
