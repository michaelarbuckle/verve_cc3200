#ifndef __VERVE_JSON_H__
#define __VERVE_JSON_H__
#define READ_SIZE           1450
#define MAX_BUFF_SIZE       1460

unsigned char SENSORS_POST_REQUEST_URI[] = 	"/sensors";
unsigned char SENSORS_TEMPLATE_A[] = "{\"uuid\":\"%s\",\"timestamp\":\"%s\",\"sensors\":[";
unsigned char SENSORS_TEMPLATE_B[] = "{\"type\":\"%s\",\"value\":\"%s\"}%s";
unsigned char SENSORS_TEMPLATE_C[] = "]}";
unsigned char SENSORS_SEPARATOR[] = ",";



unsigned char GET_token_AHPT[]  = "__SL_G_AHP";
unsigned char GET_token_ACC[]  = "__SL_G_UAC";
unsigned char GET_token_Cloud[]  = "__SL_G_CLD";
unsigned char GET_token_Control[]  = "__SL_G_CTL";
unsigned char GET_token_CO2[]  = "__SL_G_CO2";
unsigned char GET_token_CO2_Calibrate[]  = "__SL_G_C2C";
unsigned char GET_token_DateTime[]  = "__SL_G_DAT";
unsigned char GET_token_Dust[]  = "__SL_G_DST";
unsigned char GET_token_EMF[]  = "__SL_G_EMF";
unsigned char GET_token_Humidity[]  = "__SL_G_HUM";
unsigned char GET_token_NOX[]  = "__SL_G_NOX";
unsigned char GET_token_Pressure[]  = "__SL_G_PRS";
unsigned char GET_token_Radon[]  = "__SL_G_RAD";
unsigned char GET_token_RadonLogs[]  = "__SL_G_RDL";
unsigned char GET_token_TEMP[]  = "__SL_G_UTP";
unsigned char GET_token_UUID[]  = "__SL_G_UID";
unsigned char GET_token_VOC[]  = "__SL_G_VOC";
unsigned char GET_token_Logger[]  = "__SL_G_LOG";

unsigned char Name_ACC[]  = "motion";
unsigned char Name_CO2[]  = "CO2";
unsigned char Name_Dust[]  = "dust";
unsigned char Name_EMF[]  = "EMF";
unsigned char Name_Humidity[]  = "humidity";
unsigned char Name_NOX[]  = "NOX";
unsigned char Name_Pressure[]  = "pressure";
unsigned char Name_Radon[]  = "radon";
unsigned char Name_TEMP[]  = "temperature";
unsigned char Name_UUID[]  = "UUID";
unsigned char Name_VOC[]  = "VOC";

unsigned char GET_token_UIC[]  = "__SL_G_UIC";




typedef enum
{
MQTT, WEB_SVC, TIME_SERIES
} msg_format;

const char* getJSON();
const char* getSensorJSONElement(char* uuid, char* name, char*  value, char*  units, char*  timestamp);
const char* getSensorsJSONArray(char** elements);
const char* getDeviceJSON(char* uuid, char* name, char** elements);


#endif
