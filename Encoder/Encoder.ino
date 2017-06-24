/*
 Name:		Encoder.ino
 Created:	2/7/2017 8:08:33 PM
 Author:	Raphi
*/

#include <string.h>

// Encoder
const byte interruptEncLeftA = 18;
const byte interruptEncLeftB = 19;
const byte interruptEncRightA = 20;
const byte interruptEncRightB = 21;

/*---------- || Variable || ----------*/
//----- ticks
typedef struct {
	int now = 0;		// Kalibrierstellung
	int old = 0;
	int dt = 0;
}ticks_t;

//----- state sensor
typedef struct {
	int a = 0;
	int b = 0;
} state_t;

//----- time
typedef struct {
	unsigned long now = 0;
	unsigned long old = 0;
	unsigned long dt = 0;	// Differenz
}time_t;

//----- result
typedef struct {
	float ticksPerSecond = 0;
	float speed = 0;			// Geschwindigkeit
	float acceleration = 0;		// Beschleunigung
}result_t;

//----- encoder
typedef struct {
	char* name;
	char* direction;		// Zu struct/enum und fuer Ansteuerung verwenden? (gemeinsame Variable?)
	int zustand = 1;
	ticks_t ticks;
	state_t state;
	time_t time;
	result_t result;
}encoder_t;
encoder_t encLeft;
encoder_t encRight;

/*---------- || Functions || ----------*/
int getDistance(encoder_t *enc);
void encoder(encoder_t *enc, byte pinA, byte pinB);
//--- update
void update_dt_ms(encoder_t *enc);
void update_dTicks(encoder_t *enc);
void updateDirection(encoder_t *enc);
void updateResult(encoder_t *enc);
//--- print
void printDirection(encoder_t *enc);
void printDTickets(encoder_t *enc);
void printDTime(encoder_t *enc);
void printName(encoder_t *enc);
void printTickets(encoder_t *enc);
void printSpeed(encoder_t *enc);
void printZustand(encoder_t *enc);
void printAll(encoder_t *enc);

/*---------- || Functions - Update || ----------*/
void update_dTicks(encoder_t *enc) {
	enc->ticks.dt = enc->ticks.now - enc->ticks.old;
	enc->ticks.old = enc->ticks.now;
	//return enc->ticks.dt;
}

/**
 *
*/
void update_dt_ms(encoder_t *enc) {
	enc->time.now = millis();
	enc->time.dt = enc->time.now - enc->time.old;
	enc->time.old = enc->time.now;
	//return enc->time.dt;
}

void updateDirection(encoder_t *enc) {
	if (enc->result.speed < (-0.1)) {
		enc->direction = strdup("back");
	}
	else if (enc->result.speed > 0.1) {
		enc->direction = strdup("front");
	}
	else {
		enc->direction = strdup("stop");
	}
}


int getDistance(encoder_t *enc) {	// von mm in cm
	
}
/* WORKS
unsigned long get_dt_ms() {
dt_now = millis();
dt = dt_now - dt_old;
dt_old = dt_now;
return dt;
}
*/

/**
* Geschwindigkeit [Ticks / s]
*
*/
void updateResult(encoder_t *enc) {
	update_dTicks(enc);
	update_dt_ms(enc);
	//Serial.print("Ticks (float): ");Serial.println(enc->ticks.dt);
	//Serial.print("dt_ms (float): ");Serial.println(enc->time.dt);
	enc->result.ticksPerSecond = ((float)enc->ticks.dt / (float)enc->time.dt);	// [Ticks/s] OHNE *1000, weil ms ohne Koma ; !Cast : float = int / (unsigned long) ;	
	enc->result.speed = enc->result.ticksPerSecond / 2;
}

/*---------- || < Interrupt - function > || ----------*/
/**
 * Encoder parameter ticks und Zustand aktualisieren.
 *
*/
void encoder(encoder_t *enc,byte pinA,byte pinB) {
	//----- Encoder
	//State
	enc->state.a = digitalRead(pinA);	//Bei Interrupt Port lesen
	enc->state.b = digitalRead(pinB);
	//enc->state.a = digitalRead(interruptEncLeftA);
	//enc->state.b = digitalRead(interruptEncLeftB);
	//enc->state.a = digitalRead(interruptEncRightA);
	//enc->state.b = digitalRead(interruptEncRightB);

	//-- Zustand 1
	if ((enc->zustand == 1) && (enc->state.a == 0) && (enc->state.b == 1)) {
		enc->ticks.now += 1;
		enc->zustand = 2;
	}
	if ((enc->zustand == 1) && (enc->state.a == 1) && (enc->state.b == 0)) {
		enc->ticks.now -= 1;
		enc->zustand = 4;
	}
	//-- Zustand 2
	if ((enc->zustand == 2) && (enc->state.a == 1) && (enc->state.b == 1)) {
		enc->ticks.now += 1;
		enc->zustand = 3;
	}
	if ((enc->zustand == 2) && (enc->state.a == 0) && (enc->state.b == 0)) {
		enc->ticks.now -= 1;
		enc->zustand = 1;
	}
	//-- Zustand 3
	if ((enc->zustand == 3) && (enc->state.a == 1) && (enc->state.b == 0)) {
		enc->ticks.now += 1;
		enc->zustand = 4;
	}
	if ((enc->zustand == 3) && (enc->state.a == 0) && (enc->state.b == 1)) {
		enc->ticks.now -= 1;
		enc->zustand = 2;
	}
	//-- Zustand 4
	if ((enc->zustand == 4) && (enc->state.a == 0) && (enc->state.b == 0)) {
		enc->ticks.now += 1;
		enc->zustand = 1;
	}
	if ((enc->zustand == 4) && (enc->state.a == 1) && (enc->state.b == 1)) {
		enc->ticks.now -= 1;
		enc->zustand = 3;
	}
}

/*---------- || < init > || ----------*/
void initEncoder() {
	//----- | Interrupt - Pin | -----
	//--- Left
	pinMode(interruptEncLeftA, INPUT_PULLUP);
	pinMode(interruptEncLeftB, INPUT_PULLUP);		// changed
	//--- Right
	pinMode(interruptEncRightA, INPUT_PULLUP);
	pinMode(interruptEncRightB, INPUT_PULLUP);

	//----- | Interrupt Setup | -----
	//--- Left
	attachInterrupt(digitalPinToInterrupt(interruptEncLeftA), encoderLeft, CHANGE);
	attachInterrupt(digitalPinToInterrupt(interruptEncLeftB), encoderLeft, CHANGE);
	//--- Right
	attachInterrupt(digitalPinToInterrupt(interruptEncRightA), encoderRight, CHANGE);
	attachInterrupt(digitalPinToInterrupt(interruptEncRightB), encoderRight, CHANGE);
	
	//----- | Initialisierung | -----
	//--- Left
	encLeft.name = strdup("Left");
	encLeft.direction = strdup("stop");
	//--- Right
	encRight.name = strdup("Right");
	encRight.direction = strdup("stop");
}

/*---------- || < Setup > || ----------*/
// the setup function runs once when you press reset or power the board
void setup() {
	// Serial
	Serial.begin(9600);
	// Encoder
	initEncoder();

	//Motor Links
	analogWrite(6, 0);
	analogWrite(7, 30);
	//Motor Rechts
	analogWrite(8, 20);
	analogWrite(9, 0);
}

/*---------- || < Main > || ----------*/
// the loop function runs over and over again until power down or reset
void loop() {
	updateResult(&encLeft);
	updateResult(&encRight);

	printAll(&encLeft);
	printAll(&encRight);


	delay(500);
}




/*---------- || < Print > || ----------*/
void printDirection(encoder_t *enc) {
	Serial.print("Direction: \t");
	updateDirection(enc);
	Serial.println(enc->direction);
}
void printDTickets(encoder_t *enc) {
	Serial.print("dTicks: \t");
	Serial.println(enc->ticks.dt);
}
void printDTime(encoder_t *enc) {
	Serial.print("dTime: \t");
	Serial.println(enc->time.dt);
}
void printName(encoder_t *enc) {
	Serial.print("Sensor: \t");
	Serial.println(enc->name);
}
void printTickets(encoder_t *enc) {
	Serial.print("Ticks: \t");
	Serial.println(enc->ticks.now);
}
void printSpeed(encoder_t *enc) {
	Serial.print("Speed: \t");
	updateResult(enc);
	Serial.println(enc->result.speed);				// Updates dTicks & dTime
}
void printZustand(encoder_t *enc) {
	Serial.print("Zustand: \t");
	Serial.println(enc->zustand);
}
void printAll(encoder_t *enc) {
	Serial.println("----- ");
	printName(enc);
	printZustand(enc);
	printDirection(enc);
	printTickets(enc);
	printSpeed(enc);
	printDTickets(enc);
	printDTime(enc);
}

/*---------- || < ISR > || ----------*/
void encoderLeft() {
	encoder(&encLeft, interruptEncLeftA, interruptEncLeftB);
}

void encoderRight() {
	encoder(&encRight, interruptEncRightA, interruptEncRightB);
}
