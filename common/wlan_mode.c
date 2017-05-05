/*
 * wlan_mode.c
 *
 *  Created on: Apr 3, 2017
 *      Author: m
 */
// Simplelink includes
#include "simplelink.h"
//Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"

#include "timer_if.c"

// Provisioning lib include
#include "provisioning_api.h"
#include "provisioning_defs.h"
#include "common.h"

#include "wlan_mode.h"
void timeoutHandler(void);
void generalTimeoutHandler(void);
void waitmSec(_i32 timeout);


void generalTimeoutHandler(void)
{
    Timer_IF_InterruptClear(TIMERA1_BASE);
    g_TimerBTimedOut++;
}

// General waiting function using timer
void waitmSec(_i32 timeout)
{
    //Initializes & Starts timer
    Timer_IF_Init(PRCM_TIMERA1, TIMERA1_BASE, TIMER_CFG_ONE_SHOT, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA1_BASE, TIMER_A, generalTimeoutHandler);

    g_TimerBTimedOut = 0;

    Timer_IF_Start(TIMERA1_BASE, TIMER_A, timeout);

    while(g_TimerBTimedOut != 1)
    {
        // waiting...
#ifndef SL_PLATFORM_MULTI_THREADED
        _SlNonOsMainLoopTask();
#endif
    }

    //Stops timer
    Timer_IF_Stop(TIMERA1_BASE, TIMER_A);
    Timer_IF_DeInit(TIMERA1_BASE, TIMER_A);
}

void timeoutHandler(void)
{
    Timer_IF_InterruptClear(TIMERA0_BASE);
    g_TimerATimedOut++;
}


//_i8 sl_extlib_ProvEventTimeoutHdl(_u8* event, _i32 timeout)


int startProvisioning(void)
{
	long				lRetVal = -1;
	long 				FileHandle = 0;
	slExtLibProvCfg_t	cfg;
	SlFsFileInfo_t 		FsFileInfo;


//	UART_PRINT("Starting Provisioning..\n\r");


	// Enable RX Statistics
	sl_WlanRxStatStart();

	// Check if version token file exists in the device FS.
	// If not, than create a file and write the required token

	// Creating the param_product_version.txt file once
	if (SL_FS_ERR_FILE_NOT_EXISTS == sl_FsGetInfo(SL_FILE_PARAM_PRODUCT_VERSION, 0 , &FsFileInfo))
	{
		sl_FsOpen(SL_FILE_PARAM_PRODUCT_VERSION, FS_MODE_OPEN_CREATE(100, _FS_FILE_OPEN_FLAG_COMMIT), NULL, &FileHandle);
		sl_FsWrite(FileHandle, NULL, SL_PARAM_PRODUCT_VERSION_DATA, strlen(SL_PARAM_PRODUCT_VERSION_DATA));
		sl_FsClose(FileHandle, NULL, NULL, NULL);
	}

	// Creating the config result file once
	if (SL_FS_ERR_FILE_NOT_EXISTS == sl_FsGetInfo(SL_FILE_PARAM_CFG_RESULT, 0 , &FsFileInfo))
	{
		sl_FsOpen(SL_FILE_PARAM_CFG_RESULT, FS_MODE_OPEN_CREATE(100, _FS_FILE_OPEN_FLAG_COMMIT), NULL, &FileHandle);
		sl_FsWrite(FileHandle, NULL, GET_CFG_RESULT_TOKEN, strlen(GET_CFG_RESULT_TOKEN));
		sl_FsClose(FileHandle, NULL, NULL, NULL);
	}

	// Creating the param device name file once/
	if (SL_FS_ERR_FILE_NOT_EXISTS == sl_FsGetInfo(SL_FILE_PARAM_DEVICE_NAME, 0 , &FsFileInfo))
	{
		sl_FsOpen(SL_FILE_PARAM_DEVICE_NAME, FS_MODE_OPEN_CREATE(100, _FS_FILE_OPEN_FLAG_COMMIT), NULL, &FileHandle);
		sl_FsWrite(FileHandle, NULL, GET_DEVICE_NAME_TOKEN, strlen(GET_DEVICE_NAME_TOKEN));
		sl_FsClose(FileHandle, NULL, NULL, NULL);
	}

	// Creating the netlist name file once/
	if (SL_FS_ERR_FILE_NOT_EXISTS == sl_FsGetInfo(SL_FILE_NETLIST, 0 , &FsFileInfo))
	{
		sl_FsOpen(SL_FILE_NETLIST, FS_MODE_OPEN_CREATE(100, _FS_FILE_OPEN_FLAG_COMMIT), NULL, &FileHandle);
		sl_FsWrite(FileHandle, NULL, SL_SET_NETLIST_TOKENS, strlen(SL_SET_NETLIST_TOKENS));
		sl_FsClose(FileHandle, NULL, NULL, NULL);
	}

	// Initializes configuration
	cfg.IsBlocking         = 1;    //Unused
	cfg.AutoStartEnabled   = 0;
	cfg.Timeout10Secs      = PROVISIONING_TIMEOUT/10;
	cfg.ModeAfterFailure   = ROLE_STA;
	cfg.ModeAfterTimeout   = ROLE_STA;

	lRetVal = sl_extlib_ProvisioningStart(ROLE_STA, &cfg);
	ASSERT_ON_ERROR(lRetVal);

	// Wait for WLAN Event
	while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
	{
		_SlNonOsMainLoopTask();
	}

	//
	// Turn ON the RED LED to indicate connection success
	//
	//GPIO_IF_LedOn(MCU_RED_LED_GPIO);

	//wait for few moments
	MAP_UtilsDelay(80000000);

	return SUCCESS;
}

//****************************************************************************
//
//! Confgiures the mode in which the device will work
//!
//! \param iMode is the current mode of the device
//!
//! This function
//!    1. prompt user for desired configuration and accordingly configure the
//!          networking mode(STA or AP).
//!       2. also give the user the option to configure the ssid name in case of
//!       AP mode.
//!
//! \return sl_start return value(int).
//
//****************************************************************************
int ConfigureMode(int iMode, char* pcSsidName, int maxLen)
{
//    char    pcSsidName[33];
    long   lRetVal = -1;

//    UART_PRINT("Enter the AP SSID name: ");
    GetSsidName(pcSsidName,maxLen);

    lRetVal = sl_WlanSetMode(ROLE_AP);
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SSID, strlen(pcSsidName),
                            (unsigned char*)pcSsidName);
    ASSERT_ON_ERROR(lRetVal);

  //  UART_PRINT("Device is configured in AP mode\n\r");

    /* Restart Network processor */
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    // reset status bits
    CLR_STATUS_BIT_ALL(g_ulStatus);

    return sl_Start(NULL,NULL,NULL);
}

//****************************************************************************
//
//! Get Ssid name form the user over UART
//!
//! \param pcSsidName is a pointer to the array which will contain the ssid name
//!
//! This function
//!    1. gets the ssid name string over uart
//!
//! \return iRetVal is the length of the ssid(user input).
//
//****************************************************************************
static int GetSsidName(char *pcSsidName, unsigned int uiMaxLen)
{
  char ucRecvdAPDetails = 0;
  int  iRetVal = 0;
  char acCmdStore[128];
  do
  {
      ucRecvdAPDetails = 0;

      //
      // Get the AP name to connect over the UART
      //
      iRetVal = GetCmd(acCmdStore, sizeof(acCmdStore));
      if(iRetVal > 0)
      {
          // remove start/end spaces if any
          iRetVal = TrimSpace(acCmdStore);

          //
          // Parse the AP name
          //
          strncpy(pcSsidName, acCmdStore, iRetVal);
          if(pcSsidName != NULL)
          {
              ucRecvdAPDetails = 1;
              pcSsidName[iRetVal] = '\0';
          }
      }
  }while(ucRecvdAPDetails == 0);

  return(iRetVal);
}

//*****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint using SmartConfig provisioning
//!
//! Enables SmartConfig provisioning for adding a new connection profile
//! to CC3200. Since we have set the connection policy to Auto, once
//! SmartConfig is complete, CC3200 will connect automatically to the new
//! connection profile added by smartConfig.
//!
//! \param[in]                     None
//!
//! \return                        None
//!
//! \note
//!
//! \warning                    If the WLAN connection fails or we don't
//!                             acquire an IP address, We will be stuck in this
//!                             function forever.
//
//*****************************************************************************
int SmartConfigConnect()
{
    unsigned char policyVal;
    long lRetVal = -1;

    // Clear all profiles
    // This is of course not a must, it is used in this example to make sure
    // we will connect to the new profile added by SmartConfig
    //
    lRetVal = sl_WlanProfileDel(WLAN_DEL_ALL_PROFILES);
    ASSERT_ON_ERROR(lRetVal);

    //set AUTO policy
    lRetVal = sl_WlanPolicySet(  SL_POLICY_CONNECTION,
                      SL_CONNECTION_POLICY(1,0,0,0,1),
                      &policyVal,
                      1 /*PolicyValLen*/);
    ASSERT_ON_ERROR(lRetVal);

    // Start SmartConfig
    // This example uses the unsecured SmartConfig method
    //
    lRetVal = sl_WlanSmartConfigStart(0,                /*groupIdBitmask*/
                           SMART_CONFIG_CIPHER_NONE,    /*cipher*/
                           0,                           /*publicKeyLen*/
                           0,                           /*group1KeyLen*/
                           0,                           /*group2KeyLen */
                           NULL,                        /*publicKey */
                           NULL,                        /*group1Key */
                           NULL);                       /*group2Key*/
    ASSERT_ON_ERROR(lRetVal);

    // Wait for WLAN Event
    while((!IS_CONNECTED(g_ulStatus)) || (!IS_IP_ACQUIRED(g_ulStatus)))
    {
        _SlNonOsMainLoopTask();
    }
     //
     // Turn ON the RED LED to indicate connection success
     //
    // GPIO_IF_LedOn(MCU_RED_LED_GPIO);
     //wait for few moments
     MAP_UtilsDelay(80000000);
     //reset to default AUTO policy
     lRetVal = sl_WlanPolicySet(  SL_POLICY_CONNECTION,
                           SL_CONNECTION_POLICY(1,0,0,0,0),
                           &policyVal,
                           1 /*PolicyValLen*/);
     ASSERT_ON_ERROR(lRetVal);

     return SUCCESS;
}

