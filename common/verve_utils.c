#include "verve_utils.h"
#include <stdlib.h>

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <math.h>
#include "netcfg.h"
#include "simplelinklibrary.h"
#include "common_types.h"


#define GP  0x107   /* x^8 + x^2 + x + 1 */
#define DI  0x07
#define CRC16 0x8005

static void init_crc8();
void crc8(unsigned char *crc, unsigned char m);
uint16_t gen_crc16(const uint8_t *data, uint16_t size);
unsigned int calc_crc(unsigned char
*start_of_packet, unsigned char
*end_of_packet);


void getModbusCRC(const char* hexString, int* hi , int* low )
{
	unsigned int calc_crc(unsigned char
	*start_of_packet, unsigned char
	*end_of_packet);

}

char* getCRC16ForHexString(const char* hexString){return (char*)NULL;}
void calcCRC16forHex(const char* hexString, int* hi , int* low ){}
char* getMACAddressForDevice(){
	return  getMacAddress();
//	return (char*)NULL;
}
char* getUUIDForDevice(){return (char*)NULL;}
char* getUUIDForSensor(const char* deviceUUID, const char* sensorName,int itemNumber){return (char*)NULL;}

static const char theDigits[] = "0123456789";
unsigned short itoa(char cNum, char *cString)
{
    char* ptr;
    char uTemp = cNum;
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
        *ptr = theDigits[uTemp % 10];
        uTemp /= 10;
    }

    return length;
}



int keyIndex = 0;

/*
char* getMacAddress()
{


//    uint8_t macAddressVal[SL_MAC_ADDR_LEN + 2];
//    uint8_t macAddressLen = SL_MAC_ADDR_LEN + 2;
//   sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,macAddressVal);

    _u8 macAddressVal[SL_MAC_ADDR_LEN + 2];
    _u8 macAddressLen = SL_MAC_ADDR_LEN + 2;
    long slresult = sl_NetCfgGet(SL_MAC_ADDRESS_GET,NULL,&macAddressLen,(_u8 *)macAddressVal);
//Error -2001 is received when the RX buffer is less then the received data.
//8 byte buffer (SL_MAC_ADDR_LEN + 2)

        char macAddressPart[2];
    	static char macAddressFull[18]; //18

    	for (i = 0 ; i < 6 ; i++)
    	{
    		sprintf(macAddressPart, "%02X", macAddressVal[i]);
    		strcat(macAddressFull, (char *)macAddressPart);
    		strcat(macAddressFull, ":");
    	}

    	macAddressFull[17] = '\0'; // Replace the the last : with a zero termination

    	return macAddressFull;
}
*/
static unsigned char crc8_table[256];     /* 8-bit table */
static int made_table=0;

static void init_crc8()
     /*
      * Should be called before any other crc function.
      */
{
  int i,j;
  unsigned char crc;

  if (!made_table) {
    for (i=0; i<256; i++) {
      crc = i;
      for (j=0; j<8; j++)
        crc = (crc << 1) ^ ((crc & 0x80) ? DI : 0);
      crc8_table[i] = crc & 0xFF;
      /* printf("table[%d] = %d (0x%X)\n", i, crc, crc); */
    }
    made_table=1;
  }
}

void crc8(unsigned char *crc, unsigned char m)
     /*
      * For a byte array whose accumulated crc value is stored in *crc, computes
      * resultant crc obtained by appending m to the byte array
      */
{
  if (!made_table)
    init_crc8();

  *crc = crc8_table[(*crc) ^ m];
  *crc &= 0xFF;
}
//I couldn't really tell you. But it seems that if you want to append the CRC to a buffer (in order for a CRC check to result in 0), you have to append the 16 CRC value in little-endian order. Since the function operates on the data in the buffer on a byte-by-byte basis, this will be true whether the machine is big-endian or little-endian.

uint16_t gen_crc16(const uint8_t *data, uint16_t size)
{
    uint16_t out = 0;
    int bits_read = 0, bit_flag;

    /* Sanity check: */
    if(data == NULL)
        return 0;

    while(size > 0)
    {
        bit_flag = out >> 15;

        /* Get next bit: */
        out <<= 1;
        out |= (*data >> bits_read) & 1; // item a) work from the least significant bits

        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        /* Cycle check: */
        if(bit_flag)
            out ^= CRC16;

    }

    // item b) "push out" the last 16 bits
    int i;
    for (i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if(bit_flag)
            out ^= CRC16;
    }

    // item c) reverse the bits
    uint16_t crc = 0;
    i = 0x8000;
    int j = 0x0001;
    for (; i != 0; i >>=1, j <<= 1) {
        if (i & out) crc |= j;
    }

    return crc;
}
//That function returns 0xbb3d for me when I pass in "123456789".

#define POLYNOMIAL 0xA001;
unsigned int calc_crc(unsigned char
*start_of_packet, unsigned char
*end_of_packet)
{
unsigned int crc;
unsigned char bit_count;
unsigned char *char_ptr;
/* Start at the beginning of the packet */
char_ptr = start_of_packet;
/* Initialize CRC */
crc = 0xffff;
/* Loop through the entire packet */
do{
/* Exclusive-OR the byte with the CRC */
crc ^= (unsigned int)*char_ptr;
/* Loop through all 8 data bits */
bit_count = 0;
do{
/* If the LSB is 1, shift the CRC and XOR
the polynomial mask with the CRC */
if(crc & 0x0001){
crc >>= 1;
crc ^= POLYNOMIAL;
}
/* If the LSB is 0, shift the CRC only */
else{
crc >>= 1;
}
} while(bit_count++ < 7);
} while(char_ptr++ < end_of_packet);
return(crc);
}

char* stringFrmFloat(float inFlt)
{
char str[100];
//float adc_read = 678.0123;

char *tmpSign = (inFlt < 0) ? "-" : "";
float tmpVal = (inFlt < 0) ? -inFlt : inFlt;

int tmpInt1 = tmpVal;                  // Get the integer (678).
float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

// Print as parts, note that you need 0-padding for fractional bit.

int ret = sprintf (str, "%s%d.%03d", tmpSign, tmpInt1, tmpInt2);

size_t szStr = strlen(str)+1;

//return &str;
char *buf = malloc(sizeof(char)*szStr); /* allocate initial buffer */
int i;
memcpy(buf,str,szStr);

return buf;


}

// prints a number with 2 digits following the decimal place
// creates the string backwards, before printing it character-by-character from
// the end to the start
//
// Usage: myPrintf(270.458)
//  Output: 270.45
const char * floatToString(float fVal)
{
	int DECIMAL_FACTOR = 1000000;
    char result[100];
    int dVal, dec, i;

    fVal += 0.005;   // added after a comment from Matt McNabb, see below.

    dVal = fVal;
    dec = (int)(fVal * DECIMAL_FACTOR) % DECIMAL_FACTOR;


    memset(result, 0, 100);

    result[0] = (dec % 10) + '0';
    result[1] = (dec / 10) + '0';
    result[2] = '.';

    i = 3;
    while (dVal > 0)
    {
        result[i] = (dVal % 10) + '0';
        dVal /= 10;
        i++;
    }
    if (fVal < 0) {
        result[i++] ='-';
      }

    char *buf = malloc(sizeof(char)*i); /* allocate initial buffer */
    int j =0;
    for (i=strlen(result)-1; i>=0; i--, j++)
        putc(result[i], buf[j]);

    buf[j] = '\0';
    return buf;

}


const char*
format(char *pcFormat, ...)
{
    char  cBuf[256];
    va_list list;
    va_start(list,pcFormat);
    vsnprintf(cBuf,256,pcFormat,list);
    int cCnt = strlen(cBuf);
    char *buf = malloc(sizeof(char)*cCnt); /* allocate initial buffer */
    memcpy(buf,cBuf,cCnt);
    return buf;
}


const char* getDateTimeString()
{
int retVal =  getDeviceTimeDate();//Returns: On success, zero is returned. On error, -1 is returned
if (retVal == 0)
{
	return format("%d-%d-%d %d:%d.%d",dateTime.sl_tm_year,dateTime.sl_tm_mon,dateTime.sl_tm_day,dateTime.sl_tm_hour,dateTime.sl_tm_min,dateTime.sl_tm_sec);
} else
{
    return "No date";
}

}
