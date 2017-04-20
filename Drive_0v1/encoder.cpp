// 
// 
// 
#define encoder_c
#include "encoder.h"
// 
// 
// 

/* WORKS
unsigned long get_dt_ms() {
dt_now = millis();
dt = dt_now - dt_old;
dt_old = dt_now;
return dt;
}
*/

void update_dTicks(encoder_t *enc) {
	enc->ticks.dt = enc->ticks.now - enc->ticks.old;
	enc->ticks.old = enc->ticks.now;
	//return enc->ticks.dt;
}

void update_dt_ms(encoder_t *enc) {
	enc->time.now = millis();
	enc->time.dt = enc->time.now - enc->time.old;
	enc->time.old = enc->time.now;
	//return enc->time.dt;
}

void updateDirection(encoder_t *enc) {
	if (enc->result.speed < (-0.1)) {
		//enc->direction = strdup("back");
		enc->result.direction = ENCBACK;
	}
	else if (enc->result.speed > 0.1) {
		//enc->direction = strdup("front");
		enc->result.direction = ENCFRONT;
	}
	else {
		//enc->direction = strdup("stop");
		enc->result.direction = ENCSTOP;
	}
}

/**
* Geschwindigkeit [Ticks / s]
*
*/
void updateResult(encoder_t *enc) {
	updateDirection(enc);
	update_dTicks(enc);
	update_dt_ms(enc);
	//Serial.print("Ticks (float): ");Serial.println(enc->ticks.dt);
	//Serial.print("dt_ms (float): ");Serial.println(enc->time.dt);

	//DIV 0 !!!!!
	enc->result.ticksPerSecond = 100*((float)enc->ticks.dt / (float)enc->time.dt);	// [Ticks/s] OHNE *1000, weil ms ohne Koma ; !Cast : float = int / (unsigned long) ;	
																				//--- Geschwindigkeit [Ticks / s]
	enc->result.speed = enc->result.ticksPerSecond / 2;
	//--- Beschleunigung [Ticks / s^2]
	enc->result.speedOld = enc->result.speed;
	enc->result.acceleration = (enc->result.speed - enc->result.speedOld) / enc->time.dt;	//DIV 0 !!!!!
	//--- Distanz [mm]
	enc->result.distance_mm = enc->result.distance_mm + enc->umlaufdistanz_mm * (enc->ticks.dt / enc->result.ticksPerRotation); //DIV 0 !!!!!
}

/*---------- || < Interrupt - function > || ----------*/
/**
* Encoder parameter ticks und Zustand aktualisieren.
*
*/
void encoderInterrupt(encoder_t *enc, byte pinA, byte pinB) {
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

/*---------- || < Get & Set > || ----------*/
int getDistance_mm(encoder_t *enc) {	// von mm in cm
	return enc->result.distance_mm;
}

int getSpeed(encoder_t *enc) {	// von mm in cm
	return enc->result.speed;
}

float getAcceleration(encoder_t *enc) {	// von mm in cm
	return enc->result.acceleration;
}

directionEncoder_t getDirection(encoder_t *enc) {
	return enc->result.direction;
}

/*---------- || < Print > || ----------*/
void printEncName(encoder_t *enc) {
	Serial.print("Sensor: \t");
	Serial.println(enc->name);
}
void printEncDirection(encoder_t *enc) {
	Serial.print("Direction: \t");
	updateDirection(enc);
	Serial.println(enc->result.direction);
}
void printEncDTickets(encoder_t *enc) {
	Serial.print("dTicks: \t");
	Serial.println(enc->ticks.dt);
}
void printEncDTime(encoder_t *enc) {
	Serial.print("dTime: \t");
	Serial.println(enc->time.dt);
}

void printEncTickets(encoder_t *enc) {
	Serial.print("Ticks: \t");
	Serial.println(enc->ticks.now);
}
void printEncAcceleration(encoder_t *enc) {
	Serial.print("Acceleration: \t");
	Serial.println(getAcceleration(enc));
	//Serial.println(enc->result.acceleration);
}
void printEncSpeed(encoder_t *enc) {
	Serial.print("Speed: \t");
	updateResult(enc);						// Updates dTicks & dTime
	Serial.println(enc->result.speed);
}
void printEncDistance(encoder_t *enc) {
	Serial.print("Distanz: \t");
	Serial.println(getDistance_mm(enc));
	//Serial.println(enc->result.distance_mm);
}
void printEncZustand(encoder_t *enc) {
	Serial.print("Zustand: \t");
	Serial.println(enc->zustand);
}

void printEncAll(encoder_t *enc) {
	Serial.println("----- ");
	printEncName(enc);
	printEncZustand(enc);
	printEncDirection(enc);
	printEncTickets(enc);
	printEncDistance(enc);
	printEncSpeed(enc);
	printEncAcceleration(enc);
	printEncDTickets(enc);
	printEncDTime(enc);
}
