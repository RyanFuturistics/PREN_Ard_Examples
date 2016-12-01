/*
Name:		Ultrasonic_HC-SR04.ino
Created:	10/13/2016 6:50:30 PM
Author:	Raphi


---------------INFOS---------------
Misst die Distanz über den Ultraschalldistanzsensor HC-SR04.
Zusätzlich wird zurückgegeben, in welchem Bereich die Messung liegt.

-----Ausgabe-----
+Seriell
	-Distanz
	-Bereich
		- "Green" (i.O.)
		- "Red" (n.i.O.)
		- "Out of Range"

------------------------------
*/

// Initialise
#define US1_trigPin 2
#define US1_echoPin 3
//#define led1 22
//#define led2 24

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(9600);

	pinMode(US1_trigPin, OUTPUT);
	pinMode(US1_echoPin, INPUT);
	//pinMode(led1, OUTPUT);
	//pinMode(led2, OUTPUT);
}

// the loop function runs over and over again until power down or reset
void loop() {
	long duration, distance;

	Serial.println("Start:");
	for (;;) {
		//--Trigger
		//Set 10us High (LOW-HIGH-LOW)
		digitalWrite(US1_trigPin, LOW);
		delayMicroseconds(2);
		digitalWrite(US1_trigPin, HIGH);
		delayMicroseconds(10);
		digitalWrite(US1_trigPin, LOW);

		//--echoPin, 
		//Reads and returns wave travel time in microseconds
		duration = pulseIn(US1_echoPin, HIGH);		//NICHT PULSEIN BENUTZEN


		//--Calculation
		distance = (duration / 2)/29.1;				//Echo Hin & Zurück => distance/2
													//Speed of sound: 340m/s
		//--Result
		if (distance < 4) {
			Serial.println("Red \n");
		}
		else {
			Serial.println("Green");
		}
		if (distance >= 200 || distance <= 0) {
			Serial.println("Out of range");
		}
		else {
			Serial.print(distance);
			Serial.println(" cm");
		}

		//--Delay
		delay(10);
	}
}