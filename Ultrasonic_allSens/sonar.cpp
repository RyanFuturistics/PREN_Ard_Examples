// 
// 
// 

#include "sonar.h"

extern Us_ErrorCode_t us_state = US_DEFAULT;

void checkRange_mm(float distance_mm) {
	if (distance_mm < 4 && distance_mm > 0) {
		setUsErrorCode(US_NOK);
		//us_state = US_NOK;
	}
	else if (distance_mm >= 200 || distance_mm <= 0) {
		setUsErrorCode(US_OUT_OF_RANGE);
		//us_state = US_OUT_OF_RANGE;
	}
	else {
		setUsErrorCode(US_OK);
		//us_state = US_OK;
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
void printUsErrorCode(float distance_mm) {
	checkRange_mm(distance_mm);
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
static void printSonar_mm(float distance_mm) {
	Serial.print("Distance: ");
	Serial.print(" \t ");
	Serial.print(distance_mm);
	Serial.print("mm ");
}

/**
*	Ping-Messung in mm.
*/
void printSonarPing_mm(float distance_mm) {
	printSonar_mm(distance_mm);
	Serial.print("(Ping) \n");
	//printUsErrorCode(distance_mm);
}
/**
*	Median-Messung in mm.
*/
void printSonarMedian_mm(float distance_mm) {
	printSonar_mm(distance_mm);
	Serial.print("(Median) \n");
	//printUsErrorCode(distance_mm);
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
