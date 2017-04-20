/*
 Name:		Drive_0v1.ino
 Created:	4/13/2017 1:26:57 PM
 Author:	Raphi
*/

#include "PID_v1.h"
#include <string.h>

#include "encoder.h"
#include "motorDriver.h"
/*---------- || < Testing > || ----------*/
unsigned long timeSerialPrint = 0;
//unsigned long timeSerialPrintOld = 0;

/*---------- || < Encoder > || ----------*/
//@works 16-04-2017
//------ PIN
const byte interruptEncLeftA = 18;	//Enc Phase A anschliessen
const byte interruptEncLeftB = 19;	//Enc Phase B anschliessen
const byte interruptEncRightA = 21;	//Enc Phase B !!! anschliessen
const byte interruptEncRightB = 20;	//Enc Phase A !!! anschliessen
//------ Datentypen
encoder_t encLeft;
encoder_t encRight;

/*---------- || < MotorDriver > || ----------*/
//@works 16-04-2017
//------ PIN
const byte pwmDriverIn4 = 6;
const byte pwmDriverIn3 = 7;
const byte pwmDriverIn2 = 8;
const byte pwmDriverIn1 = 9;
//----- Datentypen
driver_t driverLeft;
driver_t driverRight;

/*---------- || < PID > || ----------*/
//rightPIDval & leftPIDval nicht mit typedef verbinden !!! Muehsamen verketten mit "PID rightPID" etc.
struct {
	double Setpoint, Input, Output;
	double Kp = 0.32, Ki = 0.8, Kd = 0;
	//Optimal nur Rad (17-04-17): Kp = 0.32, Ki = 0.8, Kd = 0 @Setpoint 100
	//kp = 0.3 Schwingen leicht (85-105)
	//kp = 0.35 Schwingen 35-50 @setpoint = 200
 	//kp = 0.43 Schwingen
}rightPIDval;

struct {
	double Setpoint, Input, Output;
	double Kp = 0.32, Ki = 0.8, Kd = 0;
}leftPIDval;

PID rightPID(&rightPIDval.Input, &rightPIDval.Output, &rightPIDval.Setpoint,
	rightPIDval.Kp, rightPIDval.Ki, rightPIDval.Kd, DIRECT);
PID leftPID(&leftPIDval.Input, &leftPIDval.Output, &leftPIDval.Setpoint,
	leftPIDval.Kp, leftPIDval.Ki, leftPIDval.Kd, DIRECT);

/*---------- || < Kommunikation > || ----------*/ 
//@works 16-04-2017
char cmdRaspb;
enum errorCode {
	BATTERYEMPTY	= 0,		// OPTIONAL				(Arduino)
	SENSORPROBLEM	= 1,		// Navigation			(Rasbp)
	ARDCOMMUNICATION= 2,		// Kommunikation		(Arduino)
	MOTORPROBLEM	= 3,		// Motor Fehlfunktion	(Arduino)
	SYNTAXERROR	= 4,			// Allgemein
	NOERROR = 99
};
errorCode error = NOERROR;

struct response {
	char* ok	= "ok";
	char* busy	= "busy";
};
struct {
	struct response response;
}dataSendToRaspb;


/*---------- || <  Funktionen > || ----------*/
void initDriver();
void initEncoder(encoder_t *encLeft, encoder_t *encRight);
void initPID();
//Testing
void testSerialResponse(void);
void printErrorCode(errorCode error);
//Print
void printErrorCode(errorCode error);

/*---------- || < Setup > || ----------*/
// the setup function runs once when you press reset or power the board
void setup() {
	//----- Serial
	Serial.begin(9600);

	/*---------- || INIT || ----------*/
	//------ MotorDriver
	initDriver();
	//------ Encoder
	initEncoder(&encLeft, &encRight);
	//------ PID
	initPID();

	/*---------- || INIT (END) || ----------*/
	//Test Infos
	Serial.println("----- | Test | -----");
	Serial.println("---| ResponseKeywords:");
	testSerialResponse();
}

/*---------- || < Loop > || ----------*/
// the loop function runs over and over again until power down or reset
void loop() {
	delay(20);
	/*---------- || updateEncoder || ----------*/
	updateResult(&encLeft);
	updateResult(&encRight);
	//----- Print Encoder
	//printEncAll(&encLeft);
	//printEncAll(&encRight);
	
	/*---------- || PID || ----------*/
	
	rightPIDval.Setpoint = 60;
	rightPIDval.Input = encRight.result.ticksPerSecond;
	rightPID.Compute();

	leftPIDval.Setpoint = 60;
	leftPIDval.Input = encLeft.result.ticksPerSecond;
	leftPID.Compute();

	/*---------- || motorDriver || ----------*/
	//Serial.println("MotorDriver....");
	driverLeft.direction = FORWARD;
	driverRight.direction = FORWARD;

	//----- Drive
	driverRight.valuePwm = rightPIDval.Output;	//Casting!!
	//driverRight.valuePwm = 0;
	driverLeft.valuePwm = leftPIDval.Output;	//Casting!
	//driverLeft.valuePwm = 0;
	driver(&driverRight, pwmDriverIn3, pwmDriverIn4, driverRight.valuePwm);
	driver(&driverLeft, pwmDriverIn2,pwmDriverIn1, driverLeft.valuePwm);


	/*---------- || Serial || ----------*/
	if ((millis()- timeSerialPrint)>=500) {
		timeSerialPrint = millis();

		Serial.println("-----RIGHT----------");
		//----- Encoder
		Serial.print("Setpoint:");Serial.println(rightPIDval.Setpoint);
		//----- PID
		//Serial.println("PID....");
		Serial.print("Setpoint"); Serial.println(rightPIDval.Setpoint);
		Serial.print("T/s: ");Serial.println(rightPIDval.Input);
		Serial.println(rightPIDval.Kp);
		//Serial.print("Out: ");
		//Serial.println(rightPIDval.Output);
		//Serial.print("PWM: "); Serial.println(driverRight.valuePwm);
		//----- Difference
		Serial.print("Delta: ");Serial.println(rightPIDval.Input - rightPIDval.Setpoint);

		Serial.println("-----LEFT-----");
		Serial.print("Setpoint:"); Serial.println(leftPIDval.Setpoint);
		Serial.print("T/s: "); Serial.println(leftPIDval.Input);
		Serial.print("Delta: "); Serial.println(leftPIDval.Input - leftPIDval.Setpoint);
		Serial.print("-> DELTA MM: "); Serial.println((leftPIDval.Input - leftPIDval.Setpoint)
			- (rightPIDval.Input - rightPIDval.Setpoint));//Left - Right
	}

	/*---------- || cmdRaspberryPi || ----------*/
	/*
	Receive Cmd from Raspberry Pi.
	*/
	switch (cmdRaspb) {
		/*
			M: MotorControl (Speed Left, Speed Right, Direction)
			Set Speed of the two motors left and right and direction.
		*/
		case 'M':
			//Funktionsaufruf
			
			break;
		/*
			C: RobotControl (Ticks, Speed, Direction)
			Move robot forward specified numbers of encoder ticks.
		*/
		case 'C':
			//Funktionsaufruf

			break;
		/*
			L: Left Turn (0° to 180°)
			Turn robot according to angle specified, even when moving forward.
		*/
		case 'L':
			//Funktionsaufruf
			
			break;
		/*
			R: Right Turn (0° to 180°)
			Turn robot according to angle specified, even when moving forward.
		*/
		case 'R':
			//Funktionsaufruf

			break;
		/*
			!: Emergency Stop
		*/
		case '!':
			//Funktionsaufruf

			break;
	// Default
		default:
			//
			break;

	}
}

/*---------- || < Functions - Init > || ----------*/
/*------- Driver ------*/
void initDriver() {
	pinMode(pwmDriverIn1, OUTPUT);
	pinMode(pwmDriverIn2, OUTPUT);
	pinMode(pwmDriverIn3, OUTPUT);
	pinMode(pwmDriverIn4, OUTPUT);
}

/*------- Encoder ------*/
void initEncoder(encoder_t *encLeft, encoder_t *encRight) {
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
	//----- | Initialwerte | -----
	//--- Left
	encLeft->name = strdup("Left"); //encLeft->direction = strdup("stop");
	encLeft->zustand = 1;
	encLeft->result.direction = ENCSTOP;
	encLeft->umlaufdistanz_mm = 6;
	encLeft->result.ticksPerRotation = 550;		//@todo sinnvoll?
	//--- Right
	encRight->name = strdup("Right"); //encRight->direction = strdup("stop");
	encRight->zustand = 1;
	encRight->result.direction = ENCSTOP;
	encRight->umlaufdistanz_mm = 6;
	encRight->result.ticksPerRotation = 550;	//@todo sinnvoll?
}
/*------- PID ------*/
//--- leftPID
void initPID() {
	//----- rightPID
	rightPIDval.Setpoint = 0;
	//rightPID.SetOutputLimits(double Min, double Max);
	rightPID.SetOutputLimits(0, 220);
	//rightPID.SetSampleTime(???);
	rightPID.SetMode(AUTOMATIC);		// Turn PID ON

	//----- leftPID
	leftPIDval.Setpoint = 0;
	leftPID.SetOutputLimits(0, 220);
	leftPID.SetMode(AUTOMATIC);
}



/*---------- || Functions < Drive > || ----------*/


/*---------- || Functions < Testing > || ----------*/
void testSerialResponse() {
	Serial.println(dataSendToRaspb.response.ok);
	Serial.println(dataSendToRaspb.response.busy);
}

/*---------- || Functions < Print > || ----------*/
void printErrorCode(errorCode error) {
	Serial.println("---| ErrorCode: ");
	char* errorText;
	switch (error) {
	case ARDCOMMUNICATION:
		errorText = strdup("arduino communication error");
		break;
	case BATTERYEMPTY:	//&strg = strdup("1");
		errorText = strdup("battery empty");
		break;
	case MOTORPROBLEM:
		errorText = strdup("motor error");
		break;
	case NOERROR:
		errorText = strdup("no error");
		break;
	case SENSORPROBLEM:
		errorText = strdup("sensor problem");
		break;
	case SYNTAXERROR:
		errorText = strdup("syntax error");
		break;
	default:
		errorText = strdup("no error Text :(");
		break;
	}
	Serial.println(errorText);
}
/*---------- || < ISR > || ----------*/
void encoderLeft() {
	encoderInterrupt(&encLeft, interruptEncLeftA, interruptEncLeftB);
}

void encoderRight() {
	encoderInterrupt(&encRight, interruptEncRightA, interruptEncRightB);
}
