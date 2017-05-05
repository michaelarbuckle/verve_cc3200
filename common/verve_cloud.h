/*
 * verve_cloud.h
 *
 *  Created on: Feb 26, 2017
 *      Author: m
 */

#ifndef VERVE_CLOUD_H_
#define VERVE_CLOUD_H_

#define CLOUD_HOST_NAME     "verve.house" //"<host name>"
#define CLOUD_HOST_PORT     8080


#define CONTROL_HOST_NAME	"verve.local" //"<host name>"
#define CONTROL_HOST_PORT   8080

#define PROXY_IP       	    <proxy_ip>
#define PROXY_PORT          <proxy_port>

#define POST_REQUEST_URI 	"/deviceData"
#define PUT_REQUEST_URI 	"/deviceData/%s"



typedef enum{
 /* Choosing this number to avoid overlap with host-driver's error codes */
    DEVICE_NOT_IN_STATION_MODE = -0x7D0,
    DEVICE_START_FAILED = DEVICE_NOT_IN_STATION_MODE - 1,
    INVALID_HEX_STRING = DEVICE_START_FAILED - 1,
    TCP_RECV_ERROR = INVALID_HEX_STRING - 1,
    TCP_SEND_ERROR = TCP_RECV_ERROR - 1,
    FILE_NOT_FOUND_ERROR = TCP_SEND_ERROR - 1,
    INVALID_SERVER_RESPONSE = FILE_NOT_FOUND_ERROR - 1,
    FORMAT_NOT_SUPPORTED = INVALID_SERVER_RESPONSE - 1,
    FILE_OPEN_FAILED = FORMAT_NOT_SUPPORTED - 1,
    FILE_WRITE_ERROR = FILE_OPEN_FAILED - 1,
    INVALID_FILE = FILE_WRITE_ERROR - 1,
    SERVER_CONNECTION_FAILED = INVALID_FILE - 1,
    GET_HOST_IP_FAILED = SERVER_CONNECTION_FAILED  - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

//char hostname[32];

int HTTPWriteToCloud(char* destinationHost,char* postJSON, char* uuid);


#endif /* VERVE_CLOUD_H_ */
