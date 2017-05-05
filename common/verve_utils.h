/*
//    verve_utils.c
//    Purpose: Utility functions
//
//    @author Michael Arbuckle Verve Tech
//    @version 1.1 2/10/17
*/
#ifndef __VERVE_UTIL_H__
#define __VERVE_UTIL_H__

char* getCRCForHexString(const char* hexString);
char* getMACAddressForDevice();
char* getUUIDForDevice();
char* getUUIDForSensor(const char* deviceUUID, const char* sensorName, int itemNumber);
long getRSSIForWIFIAP(const char* conneection);
long getRSSIForWIFIClient(const char* conneection);
void sniffPackets();
unsigned short itoa(char cNum, char *cString);
const char * floatToString(float fVal);
char* stringFrmFloat(float inFlt);

const char* format(char *pcFormat, ...);
const char* getDateTimeString();

//Global Variables
char g_LogBuffer[256];

#endif
