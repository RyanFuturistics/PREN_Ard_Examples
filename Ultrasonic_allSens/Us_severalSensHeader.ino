/*
 Name:		Ultrasonic_severalSens.ino
 Created:	11/10/2016 11:56:36 AM
 Author:	Wirtz Raphael
*/
//Search -Keywords
/* 
@change
@delete
@edit
@new
@example
@ignore
@priority
@since
@see
/*


*/
// the setup function runs once when you press reset or power the board
/* --------------- << | INCLUDE | >> --------------- */

// Import via "Add Library"
#include "sonar.h"
#include <NewPing.h>

/* --------------- << | CHECK | >> --------------- */
unsigned int	us_timeMeasurement[SONAR_NUM];	// Zeit pro Messung bei Ultraschallsensoren
unsigned int	us_timeMeasurementsTotal = 0;
/* --------------- (END - CHECK) --------------- */

/* --------------- << | INPUT | >> --------------- */
int buttonUS_measure = 53;
/* --------------- (END - INPUT) --------------- */
/* --------------- << | ULTRASONIC | >> --------------- */
/**
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
	//NewPing(28, 29,MAX_DISTANCE),		//Sens 4
	//NewPing(30, 31,MAX_DISTANCE),		//Sens 5
	//NewPing(32, 33,MAX_DISTANCE),		//Sens 6
};
/* ---------- Values ---------- */
int i = 0;
int nrMeasurements = 0;
//unsigned int	us_cm[SONAR_NUM];
float	us_cm[SONAR_NUM];
float	us_median_mm[SONAR_NUM];
//unsigned long	us_s[SONAR_NUM];
float	us_s[SONAR_NUM];
//unsigned long	us_median_s[SONAR_NUM];
float	us_median_s[SONAR_NUM];

/* --------------- (END - ULTRASONIC) --------------- */

/* --------------- << | DRIVER | >> --------------- */
// No Code ... not yet
/* --------------- (END - DRIVER) --------------- */

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);
	pinMode(buttonUS_measure, INPUT);
	/* CSV-Format - Header */
	//Serial.print("Sensor 1 [mm];TimeSens 1 [us]; Sensor 2 [mm];TimeSens 2 [us]; Sensor 3 [mm];TimeSens 3 [us];TimeInsg\n");
	Serial.print("MessungNr;Sensor 1[mm];Sensor 2[mm];Sensor 3[mm]\n");
}
/* --------------- << | MAIN | >>--------------- */

// the loop function runs over and over again until power down or reset
void loop() {
	delay(1500);
	while (digitalRead(buttonUS_measure) != true){
		delay(10);
	}
	nrMeasurements++;
	Serial.print(nrMeasurements);
	Serial.print(";");
	//------ << Messungen >> ------
	//Serial.print("----- Messungen -----\n");
	
	for (i = 0; i <= SONAR_NUM - 1; i++) {
		/* Ping */
		/*
		us_s[i] = sonar[i].ping();
		us_cm[i] = sonar[i].ping_cm();
		*/
		/* Median */
		//ping_median()		:	Default number of iterations, 5
		//ping_median(nbr)	:	Definded number of iterations, nbr
		us_timeMeasurement[i] = millis();								// Messung fuer die...
		us_median_s[i] = sonar[i].ping_median(MEDIAN_ITERATIONS);
		us_timeMeasurement[i] = millis() - us_timeMeasurement[i];		//... Zeit der Sensorauswertung
		us_median_mm[i] = (us_median_s[i] / US_ROUNDTRIP_CM) * 10;				//@change use mm !!!!!!!!!!!
	}

	//------ << Seriell - Messungen >> ------
	for (i = 0; i <= SONAR_NUM - 1; i++) {
		//Serial.print("------Sensor ");
		//Serial.print(i+1);
		//Serial.print("\n");
		
		/* Ping*/
		/*
		Serial.print("---Ping \t");
		printUsErrorCode(us_median_mm[i]);
		Serial.print("\n");
		printSonarPing_us(us_s[i]);
		printSonarPing_cm(us_mm[i]);
		*/

		/* Median */
		/*
		Serial.print("---Median \t");
		printUsErrorCode(us_median_mm[i]);
		printSonarMedian_us(us_median_s[i]);
		printSonarMedian_mm(us_median_mm[i]);
		*/

		/* CSV-Format */

		Serial.print(us_median_mm[i]);
		Serial.print(";");
		//Serial.print("Time: \t");
		//Serial.print(us_timeMeasurement[i]);
		//Serial.print(";");
		us_timeMeasurementsTotal += us_timeMeasurement[i];
		
	}
	/* ------ << Zeit fuer Messunen >> ------ */
	//Serial.print("Time Total: ");
	//Serial.println(us_timeMeasurementsTotal);
	Serial.print("\n");
	// Reset
	us_timeMeasurementsTotal = 0;
}

/* --------------- (END - MAIN )--------------- */

