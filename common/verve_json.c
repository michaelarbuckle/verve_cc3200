/*
 * verve_json.c
 *
 *  Created on: Feb 24, 2017
 *      Author: m
 */


// JSON Parser
#include "jsmn.h"

#include "verve_utils.h"
/**
    Returns the a JSON element for the specified sensor.
    @param uuid
    @param name
    @param value
    @param units
    @param timestamp
    @return JSON element for the specified sensor.
*/
const char* getSensorJSONElement(char* uuid, char* name,char*  value,char*  units,char*  timestamp)
{
	const char* jsonFormat = "{'uuid':'%s','name':'%s','value':'%s','units':''%s','timestamp':'%s'}";

   return format(jsonFormat,uuid,name,value,units,timestamp);
}

/**
    Returns the a JSON array of available sensors.
    @param array of JSON sensor elements
    @return JSON array.
*/
const char* getSensorsJSONArray(char** elements)
{

		return "sensors :[{ TBD}]";
}

/**
    Returns the a JSON array of available sensors.
    @param array of JSON sensor elements
    @return JSON array.
*/
const char* getDeviceDataJSON(char* uuid, char* name, char** elements)
{
return "device json TBD";
}

/**
    Returns the a JSON .
    @param array of JSON sensor elements
    @return JSON array.
*/
const char* getDeviceInfoJSON(char* uuid, char* name, char* ssid)
{
return "device json TBD";
}

