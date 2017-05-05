

#ifndef __SENSOR_API_COMMON_TYPES_H__
#define __SENSOR_API_COMMON_TYPES_H__


	/* stdint.h is a C99 supported c library.
	which is used to fixed the integer size*/
	/************************************************/
#include <stdint.h>
	/************************************************/

	/*unsigned integer types*/
	typedef uint8_t u8;/**< used for unsigned 8bit */
	typedef uint16_t u16;/**< used for unsigned 16bit */
	typedef uint32_t u32;/**< used for unsigned 32bit */
	typedef uint64_t u64;/**< used for unsigned 64bit */

	/*signed integer types*/
	typedef int8_t s8;/**< used for signed 8bit */
	typedef int16_t s16;/**< used for signed 16bit */
	typedef int32_t s32;/**< used for signed 32bit */
	/*typedef	int64_t s64;*//**< used for signed 64bit */

	typedef signed long long int s64;
	/************************************************
	 * compiler is C89 or other C standard
	************************************************/
#endif
