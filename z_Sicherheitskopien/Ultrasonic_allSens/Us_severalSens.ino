/*
 Name:		Ultrasonic_severalSens.ino
 Created:	11/10/2016 11:56:36 AM
 Author:	Wirtz Raphael
*/

// the setup function runs once when you press reset or power the board
// Import via "Add Library"
#include "sonar.h"
#include <NewPing.h>

#define SONAR_NUM		6
#define MAX_DISTANCE	200
#define MEDIAN_ITERATIONS 3
//#define PING_INTERVAL	33		//Millisekunden zwischen sensor pings (29ms als minimum).

Us_ErrorCode_t us_state = US_DEFAULT;

/*
 *	Definiert Ports der Ultraschallsensoren.
 *
 *	Command:	NewPing(TriggerPing,EchoPin,MaxDistance)
 *
 *	Referenz:	NewPingExamples, "NewPing15Sensors.pde"
*/
NewPing sonar[SONAR_NUM] = {		//Sensor object array
	//NewPing(TriggerPing,EchoPin,MaxDistance)
	NewPing(22, 23,MAX_DISTANCE),		//Sens 1
	NewPing(24, 25,MAX_DISTANCE),		//Sens 2
	NewPing(26, 27,MAX_DISTANCE),		//Sens 3
	NewPing(28, 29,MAX_DISTANCE),		//Sens 4
	NewPing(30, 31,MAX_DISTANCE),		//Sens 5
	NewPing(32, 33,MAX_DISTANCE),		//Sens 6
};

/* ---------- Values ---------- */
int i = 1;
unsigned int	us_cm[SONAR_NUM];
unsigned long	us_s[SONAR_NUM];
unsigned int	us_median_cm[SONAR_NUM];
unsigned long	us_median_s[SONAR_NUM];

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);
	
}

// the loop function runs over and over again until power down or reset
void loop() {
	delay(1000);
	
	//------ Messungen ------
	Serial.print("----- Messungen -----\n");
	
	for (i = 0; i <= SONAR_NUM - 1; i++) {
		//--- Ping
		us_s[i] = sonar[i].ping();
		us_cm[i] = sonar[i].ping_cm();
		//--- Median
		//ping_median()		:	Default number of iterations, 5
		//ping_median(nbr)	:	Definded number of iterations, nbr
		us_median_s[i] = sonar[i].ping_median(MEDIAN_ITERATIONS);
		us_median_cm[i] = us_median_s[i] / US_ROUNDTRIP_CM;
	}

	//------ Seriell ------
	for (i = 0; i <= SONAR_NUM - 1; i++) {
		Serial.print("------Sensor: ");
		Serial.print(i+1);
		Serial.print(" \n");
		Serial.print("---Ping \n");
		printSonarPing_us(us_s[i]);
		printSonarPing_cm(us_cm[i]);
		Serial.print("---Median \n");
		printSonarMedian_us(us_median_s[i]);
		printSonarMedian_cm(us_median_cm[i]);
	}
}

void checkRange_cm(unsigned int distance_cm) {
	if (distance_cm < 4) {
		us_state = US_NOK;
		setUsErrorCode(US_NOK);
	}
	if (distance_cm >= 200 || distance_cm <= 0) {
		us_state = US_OUT_OF_RANGE;
	}
	else {
		us_state = US_OK;
	}
}
/* ---------- Get & Set ---------- */
void setUsErrorCode(Us_ErrorCode_t state) {
	us_state = state;
}

Us_ErrorCode_t getUsErrorCode(void) {
	return us_state;
}

/* ---------- Print ---------- */
/**
 *	 Print Status eines USensors.
*/
void printUsErrorCode(unsigned int distance_cm) {
	checkRange_cm(distance_cm);
	switch (getUsErrorCode()) {
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

/**
 *	Print Laenge der Distanzmessung eines USensors.
*/
static void printSonar_cm(unsigned int distance_cm) {
	Serial.print("Distance: ");
	Serial.print(" \t ");
	Serial.print(distance_cm);
	Serial.print("cm ");
}

/**
*	Ping-Messung in cm.
*/
void printSonarPing_cm(unsigned int distance_cm) {
	printSonar_cm(distance_cm);
	Serial.print("(Ping)");
	printUsErrorCode(distance_cm);
}
/**
 *	Median-Messung in cm.
*/
void printSonarMedian_cm(unsigned int distance_cm) {
	printSonar_cm(distance_cm);
	Serial.print("(Median) ");
	printUsErrorCode(distance_cm);
}

/**
*	Print Zeit der Distanzmessung, eines USensors.
*/
static void printSonar_us(unsigned long time) {
	Serial.print("Time: \t\t");
	Serial.print(time);
	Serial.print("us ");
}
/**
*	Ping-Messung in us.
*/
void printSonarPing_us(unsigned long time_us) {
	printSonar_us(time_us);
	Serial.print("(Ping) \n");
}
/**
*	Median-Messung in us.
*/
void printSonarMedian_us(unsigned long time_us) {
	printSonar_us(time_us);
	Serial.print("(Median) \n");
}