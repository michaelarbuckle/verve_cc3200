/*
 * verve_cloud.c
 *
 *  Created on: Apr 13, 2017
 *      Author: m
 */
// HTTP Client lib

// SimpleLink includes
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// SimpleLink includes
#include "simplelink.h"
#include "common.h"
#include "verve_cloud.h"
#include <http/client/httpcli.h>
#include <http/client/common.h>
unsigned long  g_ulDestinationIP; // IP address of destination server
unsigned long  g_ulCloudGatewayIP = 0; //Network Gateway IP address


int HTTPPutMethod(HTTPCli_Handle httpClient, const char* destinationHost,const char* putURI, const char* putData);
int ConnectToHTTPServer(HTTPCli_Handle httpClient, const char* destinationHost);


int HTTPWriteToCloud(char* destinationHost, char* postJSON, char* uuid)
{
    HTTPCli_Struct httpClient;
    int lRetVal =0;
    lRetVal = ConnectToHTTPServer(&httpClient,destinationHost);
    if(lRetVal < 0)
    {
//    	return;
    }
    char putURI[64];
	sprintf(putURI,PUT_REQUEST_URI,uuid);
	return HTTPPutMethod(&httpClient, destinationHost, putURI, postJSON);

}





//*****************************************************************************
//
//! Function to connect to HTTP server
//!
//! \param  httpClient - Pointer to HTTP Client instance
//!
//! \return Error-code or SUCCESS
//!
//*****************************************************************************
//*****************************************************************************
//
//! Function to connect to HTTP server
//!
//! \param  httpClient - Pointer to HTTP Client instance
//!
//! \return Error-code or SUCCESS
//!
//*****************************************************************************
//#define SECURE 1
static int ConnectToHTTPServer(HTTPCli_Handle httpClient, const char* destinationHost)
{
    long lRetVal = -1;
    struct sockaddr_in addr;

#ifdef USE_PROXY
    struct sockaddr_in paddr;
    paddr.sin_family = AF_INET;
    paddr.sin_port = htons(PROXY_PORT);
    paddr.sin_addr.s_addr = sl_Htonl(PROXY_IP);
    HTTPCli_setProxy((struct sockaddr *)&paddr);
#endif

    /* Resolve HOST NAME/IP */
    lRetVal = sl_NetAppDnsGetHostByName((signed char *)destinationHost,
                                          strlen((const char *)destinationHost),
                                          &g_ulDestinationIP,SL_AF_INET);
    if(lRetVal < 0)
    {
        //ASSERT_ON_ERROR(GET_HOST_IP_FAILED);
    }

#ifdef SECURE
#define SL_SSL_CA_CERT	"/cert/tel.crt"
    struct HTTPCli_SecureParams sparams;
    /* Set secure TLS connection  */
    /* Security parameters */
    sparams.method.secureMethod = SL_SO_SEC_METHOD_TLSV1_2;
    sparams.mask.secureMask = SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA; //SL_SEC_MASK_SSL_RSA_WITH_RC4_128_SHA;;
    strncpy(sparams.cafile, SL_SSL_CA_CERT, sizeof(SL_SSL_CA_CERT));
    sparams.privkey[0] = 0;
    sparams.cert[0] = 0;
    sparams.dhkey[0] = 0;
    HTTPCli_setSecureParams(&sparams);
#endif
    /* Set up the input parameters for HTTP Connection */
    addr.sin_family = AF_INET;
    addr.sin_port = htons(CONTROL_HOST_PORT);
    addr.sin_addr.s_addr = sl_Htonl(g_ulDestinationIP);

    /* Testing HTTPCli open call: handle, address params only */
    HTTPCli_construct(httpClient);
#ifdef SECURE
    lRetVal = HTTPCli_connect(httpClient, (struct sockaddr *)&addr, HTTPCli_TYPE_TLS, NULL);
#else
    lRetVal = HTTPCli_connect(httpClient, (struct sockaddr *)&addr, 0, NULL);
#endif
    if (lRetVal < 0)
    {
        //System_printf("Connection to server failed. error(%d)\n", lRetVal);
        //System_flush();
    }
    else
    {
        //System_printf("Connection to server created successfully\n");
        //System_flush();
    }

    return 0;
}

//*****************************************************************************
//
//! \brief HTTP PUT Demonstration
//!
//! \param[in]  httpClient - Pointer to http client
//!
//! \return 0 on success else error code on failure
//!
//*****************************************************************************
static int HTTPPutMethod(HTTPCli_Handle httpClient, const char* destinationHost, const char* putURI,const char* putData)
{
    long lRetVal = 0;
    HTTPCli_Field fields[4] = {
                                {HTTPCli_FIELD_NAME_HOST, destinationHost},
                                {HTTPCli_FIELD_NAME_ACCEPT, "*/*"},
                                {HTTPCli_FIELD_NAME_CONTENT_TYPE, "application/json"},
                                {NULL, NULL}
                            };
    bool        moreFlags = 1;
    bool        lastFlag = 1;
    char        tmpBuf[4];


    /* Set request header fields to be send for HTTP request. */
    HTTPCli_setRequestFields(httpClient, fields);

    /* Send PUT method request. */
    /* Here we are setting moreFlags = 1 as there are some more header fields need to send
       other than setted in previous call HTTPCli_setRequestFields() at later stage.
       Please refer HTTP Library API documentaion @ref HTTPCli_sendRequest for more information.
    */
    moreFlags = 1;
    lRetVal = HTTPCli_sendRequest(httpClient, HTTPCli_METHOD_PUT, putURI, moreFlags);
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to send HTTP PUT request header.\n\r");
        return lRetVal;
    }

//    sprintf((char *)tmpBuf, "%d", (sizeof(PUT_DATA)-1));
    sprintf((char *)tmpBuf, "%d", (sizeof(putData)-1));

    /* Here we are setting lastFlag = 1 as it is last header field.
       Please refer HTTP Library API documentaion @ref HTTPCli_sendField for more information.
    */
    lastFlag = 1;
    lRetVal = HTTPCli_sendField(httpClient, HTTPCli_FIELD_NAME_CONTENT_LENGTH, (char *)tmpBuf, lastFlag);
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to send HTTP PUT request header.\n\r");
        return lRetVal;
    }

    /* Send PUT data/body */
    lRetVal = HTTPCli_sendRequestBody(httpClient, putData, (sizeof(putData)-1));
    if(lRetVal < 0)
    {
        UART_PRINT("Failed to send HTTP PUT request body.\n\r");
        return lRetVal;
    }

    //lRetVal = readResponse(httpClient);

    return lRetVal;
}


/*
SlDateTime_t dt;
struct HTTPCli_SecureParams sparams;

/* Set current Date to validate certificate */
/*
dt.sl_tm_day = DATE;
dt.sl_tm_mon = MONTH;
dt.sl_tm_year = YEAR;
dt.sl_tm_hour = HOUR;
dt.sl_tm_min = MINUTE;
dt.sl_tm_sec = SECOND;
sl_DevSet(SL_DEVICE_GENERAL_CONFIGURATION,
          			SL_DEVICE_GENERAL_CONFIGURATION_DATE_TIME,
                            sizeof(SlDateTime_t), (unsigned char *)(&dt));
*/
/* Security parameters */
/*
sparams.method.secureMethod = SL_SO_SEC_METHOD_TLSV1_2;
sparams.mask.secureMask  = SL_SEC_MASK_TLS_RSA_WITH_AES_256_CBC_SHA;
strncpy(sparams.cafile, SL_SSL_CA_CERT, sizeof(SL_SSL_CA_CERT));
sparams.privkey[0] = 0;
sparams.cert[0] = 0;
sparams.dhkey[0] = 0;
HTTPCli_setSecureParams(&sparams);

HTTPCli_connect(&cli, (struct sockaddr *)&addr, HTTPCli_TYPE_TLS, NULL);
*/
