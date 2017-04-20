// 
// 
// 

#include "motorDriver.h"



//void driver(driver_t driver,byte DriverInA,byte DriverInB) {
void driver(driver_t *driver, byte driverInA, byte driverInB, int value) {
	switch (driver->direction) {
	case FORWARD:
		analogWrite(driverInA, value);
		analogWrite(driverInB, 0);
		break;
	case BACKWARD:
		analogWrite(driverInA, 0);
		analogWrite(driverInB, value);
		break;
	case STOP:	// ACHTUNG: Unmittelbarer Stop
		analogWrite(driverInA, 0);
		analogWrite(driverInA, 0);
	default:
		break;
	}
}
