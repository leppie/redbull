/* Martin Thomas 4/2009 */


#include "fattime.h"
#include "rtc.h"

uint32_t get_fattime (void)
{
  uint32_t res;
	RTC_t rtc;

	RTC_GetTime( &rtc );
	
	res =  (((uint32_t)rtc.year - 1980) << 25)
			| ((uint32_t)rtc.month << 21)
			| ((uint32_t)rtc.mday << 16)
			| (uint16_t)(rtc.hour << 11)
			| (uint16_t)(rtc.min << 5)
			| (uint16_t)(rtc.sec >> 1);

	return res;
}

