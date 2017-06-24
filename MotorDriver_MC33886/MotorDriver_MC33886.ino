/*
 Name:		MotorDriver_MC33886.ino
 Created:	2/9/2017 2:52:46 PM
 Author:	Raphi

/*---------- || PIN || ----------*/
const byte adcPoti_1 = 3;

const byte pwmDriverIn4 = 6;
const byte pwmDriverIn3 = 7;
const byte pwmDriverIn2 = 8;
const byte pwmDriverIn1 = 9;

//byte pwmDriverInVal1 = 0;
//byte pwmDriverInVal2 = 0;


/*---------- || Values || ----------*/
int valuePoti_1 = 0;	// adc value between 0 and 1023 (change with analogReference)
//boolean state = 0;

typedef enum {
	FORWARD,
	BACKWARD,
	STOP
}direction_t;

typedef struct {
	int valuePwm = 0;
	direction_t direction = STOP;
}driver_t;

driver_t driverLeft;
driver_t driverRight;

/*---------- || Function || ----------*/
void initDriver();
void driver(driver_t *driver, byte driverInA, byte driverInB, int value);

/*---------- || Init - Driver || ----------*/
void initDriver() {
	pinMode(pwmDriverIn1, OUTPUT);
	pinMode(pwmDriverIn2, OUTPUT);
	pinMode(pwmDriverIn3, OUTPUT);
	pinMode(pwmDriverIn4, OUTPUT);
}

//void driver(driver_t driver,byte DriverInA,byte DriverInB) {
void driver(driver_t *driver,byte driverInA,byte driverInB,int value) {
	switch (driver->direction) {
	case FORWARD:
		analogWrite(driverInA, value);
		analogWrite(driverInB, 0);
		break;
	default:
		break;
	}
}


// the setup function runs once when you press reset or power the board
/*---------- || Init || ----------*/
void setup() {
	Serial.begin(9600);
	Serial.println("Start...");

	//initPin();
	initDriver();

}

// the loop function runs over and over again until power down or reset
void loop() {
	driverLeft.direction = FORWARD;
	driverRight.direction = FORWARD;
	//----- HARDCODING -----
	//valuePoti_1 = analogRead(adcPoti_1);
	//valuePwm = valuePoti_1 / 4;		// analogRead (0...1023) and analogWrite (0...255

	driverLeft.valuePwm = 60;
	driverRight.valuePwm = 120;

	driver(&driverLeft, pwmDriverIn3, pwmDriverIn4, driverLeft.valuePwm);
	driver(&driverRight, pwmDriverIn1, pwmDriverIn2, driverRight.valuePwm);

	//-----
	/*
	switch (driverLeft.direction) {
		case FORWARD:
			analogWrite(pwmDriverIn1, valuePwm);analogWrite(pwmDriverIn2, 0);
			analogWrite(pwmDriverIn3, valuePwm);analogWrite(pwmDriverIn4, 0);
			//driverLeft.direction = STOP;
			break;
		case BACKWARD:
			analogWrite(pwmDriverIn1, 0);
			analogWrite(pwmDriverIn2, valuePwm);
			//driverLeft.direction = STOP;
			break;
		case STOP:
			analogWrite(pwmDriverIn1, 0);
			analogWrite(pwmDriverIn2, 0);
			// Change direction
			if (state) {
				//driverLeft.direction = FORWARD;
			}
			else {
				//driverLeft.direction = BACKWARD;
			}
			state = !state;
			break;
		default:
			break;
	}
	*/
	delay(1000);
}