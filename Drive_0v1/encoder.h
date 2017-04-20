// encoder.h

#ifdef encoder_c
#define EXTERN
#else
#define EXTERN extern
#endif

#ifndef _ENCODER_h
#define _ENCODER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

/* --------------- || < encoder.h > || --------------- */
/*---------- || Variable || ----------*/
typedef enum direction {
	ENCBACK,
	ENCFRONT,
	ENCSTOP
}directionEncoder_t;

//----- ticks
typedef struct ticks_ {
	int now = 0;		// Kalibrierstellung
	int old = 0;
	int dt = 0;
}ticks_t;

//----- state sensor
typedef struct state_ {
	int a = 0;
	int b = 0;
}state_t;

//----- result
typedef struct result_ {
	directionEncoder_t direction;
	float ticksPerSecond = 0;
	float ticksPerRotation = 550;
	float timePerRotation = 0;
	float distance_mm = 0;
	float speed = 0;			// Geschwindigkeit
	float speedOld = 0;
	float acceleration = 0;		// Beschleunigung
}result_t;

//----- time
typedef struct time_ {
	unsigned long now = 0;
	unsigned long old = 0;
	unsigned long dt = 0;	// Differenz
}time_t;

//----- encoder
typedef struct encoder_ {
	char* name;
	//char* direction;		// Zu struct/enum und fuer Ansteuerung verwenden? (gemeinsame Variable?)
	int zustand;
	int umlaufdistanz_mm = 6;
	ticks_t ticks;
	state_t state;
	time_t time;
	result_t result;
}encoder_t;

/*---------- || Functions || ----------*/
int getDistance(encoder_t *enc);
void encoderInterrupt(encoder_t *enc, byte pinA, byte pinB);
void initEncoder();
//--- update
void update_dt_ms(encoder_t *enc);
void update_dTicks(encoder_t *enc);
void updateDirection(encoder_t *enc);
void updateResult(encoder_t *enc);
//--- print
void printEncDirection(encoder_t *enc);
void printEncDTickets(encoder_t *enc);
void printEncDTime(encoder_t *enc);
void printEncName(encoder_t *enc);
void printEncTickets(encoder_t *enc);
void printEncSpeed(encoder_t *enc);
void printEncZustand(encoder_t *enc);
void printEncAll(encoder_t *enc);

#endif