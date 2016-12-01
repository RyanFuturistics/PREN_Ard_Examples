// sonar.h

#ifndef _SONAR_h
#define _SONAR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif
/* --------------- sonar.h --------------- */
#define SONAR_NUM		6
#define MAX_DISTANCE	200
#define MEDIAN_ITERATIONS 3
//#define PING_INTERVAL	33		//Millisekunden zwischen sensor pings (29ms als minimum).

typedef enum Us_ErrorCode { 
	US_OK,
	US_NOK,
	US_OUT_OF_RANGE,
	US_DEFAULT 
}Us_ErrorCode_t;
//enum Us_ErrorCode { US_OK, US_NOK, US_OUT_OF_RANGE, US_DEFAULT };

/* ---------- Funktionen ---------- */
void checkRange_cm(unsigned int distance_cm);
//
/* ---------- Get & Set ---------- */
//
void setUsErrorCode(Us_ErrorCode_t state);
//Us_ErrorCode_t getUsErrorCode(void);

int getSonar_Num(void);
//
/* ---------- Print ---------- */
//
//static void printSonar_cm(unsigned int distance_cm);
void printUsErrorCode(unsigned int distance_cm);
void printSonarPing_cm(unsigned int distance_cm);
void printSonarMedian_cm(unsigned int distance_cm);
//static void printSonar_us(unsigned long time);
void printSonarPing_us(unsigned long time_us);
void printSonarMedian_us(unsigned long time_us);

