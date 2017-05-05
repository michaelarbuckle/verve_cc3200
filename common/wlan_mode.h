// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    LAN_CONNECTION_FAILED = -0x7D0,
    DEVICE_NOT_IN_STATION_MODE = LAN_CONNECTION_FAILED - 1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

#define WLAN_DEL_ALL_PROFILES   0xFF


#define SL_PARAM_PRODUCT_VERSION_DATA 	"R1.0"
#define PROVISIONING_TIMEOUT            300 //Number of seconds to wait for provisioning completion

volatile unsigned long  g_ulStatus = 0;//SimpleLink Status

unsigned long 	g_ulTimerA2Base;
_u8 volatile g_TimerATimedOut;
_u8 volatile g_TimerBTimedOut;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
int startProvisioning();
int ConfigureMode(int iMode, char* pcSsidName, int maxLen);
