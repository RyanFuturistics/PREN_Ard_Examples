/*
 Name:		Sketch2.ino
 Created:	10/28/2016 11:49:25 AM
 Author:	Raphi
*/

// Input
/*
* USE ENUM !!!!
*/
#define M_OUT_CW_CCW		23	// L297 Pin 17
#define M_OUT_CLOCK			25	// L297 Pin 18
#define M_OUT_STEP_HS_FS	27	// L297 Pin 19
#define M_OUT_RESET			29	// L297 Pin 20
#define M_OUT_ENABLE		31	// L297 Pin 10
#define M_OUT_CONTROL		33	// L297 Pin 11
// Output
#define M_IN_SNYC			53	// L297 Pin 11
#define M_IN_HOME			51	// L297 Pin 13

// the setup function runs once when you press reset or power the board
void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(9600);
	Serial.write("Stepper Motor\n");

	//----- Configuration L297 -----
	//ENABLE
	digitalWrite(M_OUT_ENABLE, HIGH);	/* ENABLE */
	//CW_CCW
	digitalWrite(M_OUT_CW_CCW, HIGH);	/* CW */
	//RESET (LOW activ)
	digitalWrite(M_OUT_RESET, HIGH);	/* NO Reset*/
	//Step HALF(HIGH) / FULL(LOW)		/* FULL STEP */
	digitalWrite(M_OUT_STEP_HS_FS, LOW);

	//CONTROL
	/*	Chopperbetrieb wird gesteuert. Bei Anliegen von 5 V langsames Abfallen der Phasenspannung, bei
	*	Anliegen von 0 V dynamisches Abfallen der Phasenspannung.
	*/
	digitalWrite(M_OUT_CONTROL, LOW);	//LOW or HIGH

	//----- Serial -----
	Serial.write("Start: \n");

}

// the loop function runs over and over again until power down or reset
void loop() {

	//--- Signalisieren
	for (;;) {
		
		digitalWrite(M_OUT_CLOCK, HIGH);	// Stepper
		digitalWrite(LED_BUILTIN, HIGH);	// LED blink
		delay(10);

		digitalWrite(M_OUT_CLOCK, LOW);		// Stepper
		digitalWrite(LED_BUILTIN, LOW);		// LED blink
		
		delay(2);						// wait for a second

		//--- Serial
		Serial.write("run stepper\n");
	}
}
