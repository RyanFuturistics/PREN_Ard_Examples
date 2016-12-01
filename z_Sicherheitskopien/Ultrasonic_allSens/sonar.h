// sonar.h

#ifndef _SONAR_h
#define _SONAR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif

typedef enum Us_ErrorCode { US_OK, US_NOK, US_OUT_OF_RANGE,US_DEFAULT}Us_ErrorCode_t;
//enum Us_ErrorCode { US_OK, US_NOK, US_OUT_OF_RANGE, US_DEFAULT };
