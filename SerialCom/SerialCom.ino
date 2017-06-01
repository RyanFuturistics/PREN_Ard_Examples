/* Serial Test Code*/

//String inputString = "";         // a string to hold incoming data
#include <SoftwareSerial.h>
int inputInt = 0;
boolean inputComplete = false;  // whether the input is complete

void setup() {
	// initialize serial:
	Serial.begin(9600);
	// reserve 200 bytes for the inputString:
	//inputString.reserve(200);
}

void loop() {
	// print the string when a newline arrives:
	if (inputComplete) {
		Serial.println(inputInt);
		// clear the string:
		//inputString = "";
		inputInt = 0;
		inputComplete = false;
	}
}

/*
SerialEvent occurs whenever a new data comes in the
hardware serial RX.  This routine is run between each
time loop() runs, so using delay inside loop can delay
response.  Multiple bytes of data may be available.
Only Int values.
*/
void serialEvent() {
	while (Serial.available()) {
		// get the new byte:
		char inChar = (char)Serial.read();
		// add it to the inputInt:
		if (inChar != '\n') {
			inputInt *= 10;
			inChar -= 48; //sub asci '0'
			inputInt += inChar;
		}
		// if the incoming character is a newline, set a flag
		// so the main loop can do something about it:
		if (inChar == '\n') {
			inputComplete = true;
		}
	}
}