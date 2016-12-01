/*
 Name:		LED_PWM.ino
 Created:	10/13/2016 6:50:30 PM
 Author:	Raphi
*/

// Initialise
int pwmPin1 = 2;



// the setup function runs once when you press reset or power the board
void setup() {
	pinMode(LED_BUILTIN, OUTPUT);
	Serial.begin(9600);
	
}

// the loop function runs over and over again until power down or reset
void loop() {
	Serial.write("Start:");
	analogWrite(pwmPin1, 127);	//PWM 50% Duty Cyle	
	for(;;){

		//Blink LED
		digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
		delay(1000);                       // wait for a 
		digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
		delay(1000);                       // wait for a second

		//Serial
		Serial.write("test\n");
	}

}
