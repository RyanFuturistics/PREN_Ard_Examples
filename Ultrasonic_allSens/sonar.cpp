// 
// 
// 

#include "sonar.h"

extern Us_ErrorCode_t us_state = US_DEFAULT;

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

/* --------------- << | GET & SET | >> --------------- */
void setUsErrorCode(Us_ErrorCode_t state) {
	us_state = state;
}

Us_ErrorCode_t getUsErrorCode(void) {
	return us_state;
}

int getSonar_Num(void) {
	return SONAR_NUM;
}

/* --------------- << | PRINT | >> --------------- */
/**
*	 Print Status eines USensors.
*/
void printUsErrorCode(unsigned int distance_cm) {
	checkRange_cm(distance_cm);
	switch (getUsErrorCode()) {
	case US_NOK:
		Serial.print("\t NOK");
		break;
	case US_OK:
		Serial.print("\t OK");
		break;
	case US_OUT_OF_RANGE:
		Serial.print("\t Out Of Range");
		break;
	default:
		Serial.print("default \n");
		break;
	}
	Serial.print("\n");
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
	Serial.print("(Ping) \n");
	//printUsErrorCode(distance_cm);
}
/**
*	Median-Messung in cm.
*/
void printSonarMedian_cm(unsigned int distance_cm) {
	printSonar_cm(distance_cm);
	Serial.print("(Median) \n");
	//printUsErrorCode(distance_cm);
}

/**
*	Print Zeit der Distanzmessung, eines USensors.
*/
static void printSonar_us(unsigned long time) {
	Serial.print("Echo: \t\t");
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
