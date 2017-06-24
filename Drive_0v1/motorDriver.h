// motorDriver.h

#ifndef _MOTORDRIVER_h
#define _MOTORDRIVER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

/* --------------- || < motorDriver.h > || --------------- */
/*---------- || Variable || ----------*/
typedef enum {
	FORWARD,
	BACKWARD,
	STOP
}directionDriver_t;

typedef struct {
	int valuePwm = 0;
	directionDriver_t direction = STOP;
}driver_t;

/*---------- || Functions || ----------*/
void driver(driver_t *driver, byte driverInA, byte driverInB, int value);

#endif

