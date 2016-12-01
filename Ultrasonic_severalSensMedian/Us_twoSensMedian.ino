/*
 Name:		Ultrasonic_severalSens.ino
 Created:	11/10/2016 11:56:36 AM
 Author:	Raphi
*/

// the setup function runs once when you press reset or power the board
// Import via "Add Library"
#include "sonar.h"
#include <NewPing.h>

#define SONAR_NUM		3
#define MAX_DISTANCE	200
#define MEDIAN_ITERATIONS 3

//#define PING_INTERVAL	33		//Millisekunden zwischen sensor pings (29ms als minimum).

Us_ErrorCode_t us_state = US_DEFAULT;
/* 
 *	
 *	Referenz:	NewPingExamples, "NewPing15Sensors.pde"
*/
NewPing sonar[SONAR_NUM] = {		//Sensor object array
	//NewPing(TriggerPing,EchoPin,MaxDistance)
	NewPing(22, 23,MAX_DISTANCE),
	NewPing(24, 25,MAX_DISTANCE),
	NewPing(26, 27,MAX_DISTANCE),
};
// Values
unsigned int us_cm[SONAR_NUM];
unsigned long us_s[SONAR_NUM];
unsigned int us_median_cm;
unsigned long us_median_s;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);
}

// the loop function runs over and over again until power down or reset
void loop() {
	//Transfer to "setup()"???
	//-----
	delay(1000);
	
	//------ Messungen ------
	//--- Ping
	us_s[0] = sonar[0].ping();
 	us_cm[0] = sonar[0].ping_cm();

	//--- Median
	//Default: 5 iterations
	us_median_s = sonar[0].ping_median(MEDIAN_ITERATIONS);
	us_median_cm = us_median_s / US_ROUNDTRIP_CM;
	us_s[0] = sonar[0].ping();
	//------ Seriell ------
	printf("--- Ping\n");
	printSonarPing_s(us_s[0]);
	printSonarPing_cm(us_cm[0]);
	printf("--- Median\n");
	printSonarMedian_s(us_median_s);
	printSonarMedian_cm(us_median_cm);
	
	Serial.print("----- -----\n");
}

void checkRange_cm(unsigned int distance_cm) {
	if (distance_cm < 4) {
		us_state = US_NOK;
		setUsError_Check(US_NOK);
	}
	if (distance_cm >= 200 || distance_cm <= 0) {
		us_state = US_OUT_OF_RANGE;
	}
	else {
		us_state = US_OK;
	}
}

void setUsError_Check(Us_ErrorCode_t state) {
	us_state = state;
}

Us_ErrorCode_t getUsError_Check(void) {
	return us_state;
}

/* ---------- Print ---------- */
void printUsErrorCode(unsigned int distance_cm) {
	checkRange_cm(distance_cm);
	switch (getUsError_Check()) {
		case US_NOK:
			Serial.print("\t NOK \n");
			break;
		case US_OK:
			Serial.print("\t OK \n");
			break;
		case US_OUT_OF_RANGE:
			Serial.print("\t Out Of Range\n");
			break;
		default:
			Serial.print("default \n");
			break;
	}
}

static void printSonar_cm(unsigned int distance_cm) {
	Serial.print("Distance: ");
	Serial.print(" \t ");
	Serial.print(distance_cm);
	Serial.print("cm ");
	//us_state = checkRange_cm(distance_cm);
}

void printSonarPing_cm(unsigned int distance_cm) {
	printSonar_cm(distance_cm);
	Serial.print("(Ping)");
	printUsErrorCode(distance_cm);
}

void printSonarMedian_cm(unsigned int distance_cm) {
	printSonar_cm(distance_cm);
	Serial.print("(Median) ");
	printUsErrorCode(distance_cm);
}

static void printSonar_us(unsigned long time) {
	Serial.print("Time: \t\t");
	Serial.print(time);
	Serial.print("us ");
}

void printSonarPing_s(unsigned long time_us) {
	printSonar_us(time_us);
	Serial.print("(Ping) \n");
}

void printSonarMedian_s(unsigned long time_us) {
	printSonar_us(time_us);
	Serial.print("(Median) \n");
}