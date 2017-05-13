/*
 Name:		Drive_0v1.ino
 Created:	4/13/2017 1:26:57 PM
 Author:	Raphi
*/

#include "PID_v1.h"
#include <string.h>

#include "encoder.h"
#include "motorDriver.h"
/*---------- || < Flags > || ----------*/
boolean debugFlag = 0;	//0:Send NO data to PC 1:Send data to PC via Serial

/*---------- || < enum > || ----------*/
enum color_ {
	BLANK,
	RED,
	GREEN,
	BLUE
};

/*---------- || < Mainboard - GPIO > || ----------*/
//------ RGB
const byte rgbLeftRed = 22;
const byte rgbLeftBlue = 23;
const byte rgbLeftGreen = 24;
const byte rgbRightRed = 27;//27
const byte rgbRightBlue = 26;//
const byte rgbRightGreen = 25;

typedef struct rgb_t{
	byte red;
	byte blue;
	byte green;
};
rgb_t rgbLeft;
rgb_t rgbRight;

//------ Taster
const byte tasterRight = 28;

/*---------- || < Mainboard - Encoder > || ----------*/
//@works 16-04-2017
//------ PIN
const byte interruptEncLeftA = 19;	//Enc Phase B (!) anschliessen
const byte interruptEncLeftB = 18;	//Enc Phase A (!) anschliessen
const byte interruptEncRightA = 3;	//Enc Phase A anschliessen		//Vorher 21
const byte interruptEncRightB = 2;	//Enc Phase B anschliessen		//Vorher 20
//------ Datentypen
encoder_t encLeft;
encoder_t encRight;

/*---------- || < Driver > || ----------*/
//@works 16-04-2017
//------ PIN
const byte pwmDriverIn4 = 9;
const byte pwmDriverIn3 = 8;
const byte pwmDriverIn2 = 7;
const byte pwmDriverIn1 = 6;
//----- Datentypen
driver_t driverLeft;
driver_t driverRight;

/*---------- || < PID > || ----------*/
//rightPIDval & leftPIDval nicht mit typedef verbinden !!! Muehsamen verketten mit "PID rightPID" etc.
struct {
	double Setpoint, Input, Output;
	double Kp = 0.28, Ki = 0.8, Kd = 0.004;
	//Werte Bereich => Kp: 0.25-0.32, Ki = 0.6-1.2, Kd = 0-0.0008 @delay min.20ms (Polling)
	//Optimal nur Rad (17-04-17): Kp = 0.32, Ki = 0.8, Kd = 0 @Setpoint 100	//SCHWINGEN BEI RUECKSPRUNG AUF "0"
	//kp = 0.3 Schwingen leicht (85-105)
	//kp = 0.35 Schwingen 35-50 @setpoint = 200
 	//kp = 0.43 Schwingen
}rightPIDval;

struct {
	double Setpoint, Input, Output;
	double Kp = 0.28, Ki = 0.8, Kd = 0.004;
}leftPIDval;

PID rightPID(&rightPIDval.Input, &rightPIDval.Output, &rightPIDval.Setpoint,
	rightPIDval.Kp, rightPIDval.Ki, rightPIDval.Kd, DIRECT);
PID leftPID(&leftPIDval.Input, &leftPIDval.Output, &leftPIDval.Setpoint,
	leftPIDval.Kp, leftPIDval.Ki, leftPIDval.Kd, DIRECT);

/*---------- || < cmdNavigation > || ----------*/
/*------ | Values | ------*/
int degreeTicks_ = 0;
int degree_ = 0;
struct {
	// From Raspberry
	int direction = 0;		//0 bis 4
	int rightValue = 0;
	int leftValue = 0;
	int leftIncrement = 0;	//n increments zum fahren
	int rightInrement = 0;	//""
	int leftDegree = 0;		//Winkel zum fahren
	int rightDegree = 0;	//""
	//
	int encRightOld = 0;
	int encLeftOld = 0;
}cmdNavigationValues;
/*------ | enum | ------*/
enum cmd {
	NOCMD,
	CONTROL,		// Move sepcific ticks
	EMERGENCY,		// Emergency Stop
	INFORMATION,	// Send Information
	TURNLEFT,
	TURNLEFT90,
	TURNRIGHT,
	TURNRIGHT90,
};
cmd cmdNavigation = NOCMD;

/*---------- || < Kommunikation > || ----------*/ 
//@works 16-04-2017
boolean printInformationFlag = 1;
char cmdRaspb;
String s1, s2, s3; //Buffers for serial protocol

/*---------- || < Polling > || ----------*/
unsigned long tSerialPcRefresh = 0;
unsigned long tSerialRbRefresh = 0;
unsigned long tControl = 0;

enum errorCode {
	NOERROR			= 0,
	BATTERYEMPTY	= 1,		// OPTIONAL				(Arduino)
	SENSORPROBLEM	= 2,		// Navigation			(Rasbp)
	ARDCOMMUNICATION= 3,		// Kommunikation		(Arduino)
	MOTORPROBLEM	= 4,		// Motor Fehlfunktion	(Arduino)
	SYNTAXERROR	= 5		// Allgemein
};
errorCode error = NOERROR;

enum status {
	OK		= 0,
	BUSY	= 1,
	UNDEF = 9
	//char* ok	= "ok";
	//char* busy	= "busy";
};
status statusArduino = OK;

//@TODO delete struct datasendtoRaspb
struct {
	// SEND ENUM VALUE
	//char* ok = "ok";
	//char* busy = "busy";
}dataSendToRaspb;


/*---------- || <  Funktionen > || ----------*/
void initDriver();
void initEncoder(encoder_t *encLeft, encoder_t *encRight);
void initMainboardGPIO();
void initPID();
//----- RGB
void rgbSet(rgb_t rgb, color_ color);
//----- Testing
void testSerialResponse(void);
void printErrorCode(errorCode error);
//----- Serial
void printErrorCode(errorCode error);
//-- Debug
void printPIDintegral();
void printPIDvalue();
//-- Raspbberry Pi
void printInformation(status status, errorCode error);
void serialEvent();

/*---------- || < Setup > || ----------*/
// the setup function runs once when you press reset or power the board
void setup() {
	/*---------- || Serial || ----------*/
	Serial.setTimeout(100);
	Serial.begin(38400);
	if (debugFlag == 1) {		// Debugging
		Serial.println("---------------");
		Serial.println("---DEBUGGING---");
	}

	/*---------- || INIT || ----------*/
	//------ MotorDriver
	initDriver();
	//------ Encoder
	initEncoder(&encLeft, &encRight);
	//------ LED
	initMainboardGPIO();
	//------ PID
	initPID();
	/*---------- || INIT (END) || ----------*/

	//Test Infos
	if (debugFlag == 1) {
		/*
		Serial.println("----- | Test | -----");
		Serial.println("---| ResponseKeywords:");
		testSerialResponse();
		*/
	}	
}

/*---------- || < Loop > || ----------*/
// the loop function runs over and over again until power down or reset
void loop() {
	/*---------- || Kommunikation || ----------*/
	/*----- || Debuging || -----*/
	 //----- Arduino => PC
	if (debugFlag == 1) {
		if ((millis() - tSerialPcRefresh) >= 3000) {
			tSerialPcRefresh = millis();
			//printEncAll(&encLeft); printEncAll(&encRight);
			printcmdNavigationValues();
			printErrorCode(error);
			error = NOERROR;		// Testing
			printPIDvalue();
			Serial.println("----------");
			//printPIDintegral();
		}
	}
	/*----- || Rasbperry PI || -----*/
	//----- Arduino => Raspberry PI		NOT USED
	if (debugFlag == 0) {
		if((millis() - tSerialRbRefresh) >= 100) {
		}
	}
	/*----- || Commands || -----*/
	//----- Raspberry Pi,PC => Arduino
	if (Serial.available()) {
		serialEvent();
	}
	/*---------- || Taster || ----------*/
	if ((digitalRead(tasterRight)) == HIGH) {
		statusArduino = BUSY;
		/*----- Drive forward
		rightPIDval.Setpoint = 150;
		leftPIDval.Setpoint = 150;
		setDriverDirection(1);	//Forward
		*/
		/*----- Drive stop
		leftPIDval.Setpoint = 0;
		rightPIDval.Setpoint = 0;
		*/
		/*----- Turn Left 90 */
 		cmdNavigation = TURNLEFT90;
		cmdNavigationValues.encRightOld = encRight.ticks.now;
		rgbSet(rgbLeft, BLUE);
	}
	else {
		rgbSet(rgbLeft, BLANK);
		//rightPIDval.Setpoint = 0;
		//leftPIDval.Setpoint = 0;
	}
	
	/*---------- || Control || 
	* -> Polling
	* Ablauf:	Encoder => PID => Driver
	*/
	if ((millis() - tControl) > 20) {	// 30 works
		tControl = millis();
		
		/*---------- || updateEncoder || ----------*/
		updateResult(&encLeft); updateResult(&encRight);

		/*---------- || PID || ----------*/
		//----- right
		rightPIDval.Input = abs(encRight.result.ticksPerSecond);		//@New abs()
		rightPID.Compute();
		//----- left
		leftPIDval.Input = abs(encLeft.result.ticksPerSecond);			//@New abs()
		leftPID.Compute();
		
		/*---------- || motorDriver || ----------*/
		//----- PID => Driver
		driverRight.valuePwm = rightPIDval.Output;	//Casting!!
		//driverRight.valuePwm = 0;
		driverLeft.valuePwm = leftPIDval.Output;	//Casting!
		//driverLeft.valuePwm = 0
		//----- Driver
		driver(&driverRight, pwmDriverIn3, pwmDriverIn4, driverRight.valuePwm);
		driver(&driverLeft, pwmDriverIn2, pwmDriverIn1, driverLeft.valuePwm);
	}

	//
	if (statusArduino == OK) {
		/*---------- || cmdRaspberryPi || ----------*/
		/* Raspberry Pi => Arduino
		* Receive Cmd from Raspberry Pi.
		*/
		rgbSet(rgbRight, GREEN);
		printInformationFlag = 1;
		switch (cmdRaspb) {
		/*
		E: Reset Error
		*/
		case 'E':
			error = NOERROR;
			break;
		/*
		M: MotorControl (Speed Left, Speed Right, Direction)
		Set Speed of the two motors left and right and direction.
		*/
		case 'M':
			//Funktionsaufruf
			leftPIDval.Setpoint = cmdNavigationValues.leftValue;
			rightPIDval.Setpoint = cmdNavigationValues.rightValue;
			setDriverDirection(cmdNavigationValues.direction);
			break;
		/*
		C: RobotControl (Ticks, Speed, Direction)
		Move robot forward specified numbers of encoder ticks.
		*/
		case 'C':
			//Funktionsaufruf

			break;
		/*
		* I: Information
		* Sendback Information
		*/
		case 'I':
			//Informationen nach switch-case
			break;
		/*
		L: Left Turn (0° to 180°)
		Turn robot according to angle specified, even when moving forward.
		*/
		case 'L':
			statusArduino = BUSY;
			cmdNavigation = TURNLEFT;
			cmdNavigationValues.encRightOld = encRight.ticks.now;
			// 90° => 1888 i.O. @ Setpoint left/right = 100
			if (cmdNavigationValues.leftDegree > 360) {	degree_ = 0;}
			else {degree_ = cmdNavigationValues.leftDegree;	
			}
			degreeTicks_ = (int) (1888 / 90) * degree_;
			break;
		/*
		R: Right Turn (0° to 180°)
		Turn robot according to angle specified, even when moving forward.
		*/
		case 'R':
			statusArduino = BUSY;
			cmdNavigation = TURNRIGHT;	
			//cmdNavigationValues.encRightOld = encRight.ticks.now;
			cmdNavigationValues.encLeftOld = encLeft.ticks.now;
			// 90° => 1888 i.O. @ Setpoint left/right = 100
			if (cmdNavigationValues.rightDegree > 360) { degree_ = 0; }
			else {
				degree_ = cmdNavigationValues.rightDegree;
			}
			degreeTicks_ = (1888 / 90) * degree_;
			break;
		/*
		!: Emergency Stop
		*/
		case 'S':
			//Funktionsaufruf
			leftPIDval.Setpoint = 0;		//l_pwm
			rightPIDval.Setpoint = 0;		//r_pwm	
			rightPID.SetITerm(0);
			leftPID.SetITerm(0);
			setDriverDirection(cmdNavigationValues.direction);
			break;
		// Default
		default:
			//
			printInformationFlag = 0;
			break;
		}
		cmdRaspb = 0;	// !Rücksetzten

		if (printInformationFlag == 1) {
			printInformation(statusArduino, error);
		}
		
	}

	else if(statusArduino == BUSY) {
		/*---------- || cmd Abarbeitung || ----------*/
		rgbSet(rgbRight, BLUE);
		printInformationFlag = 0;

		switch (cmdNavigation) {
		case NOCMD:statusArduino = OK; break;
		case CONTROL: break;
		case EMERGENCY: break;
		case INFORMATION: break;	//NOT USED HERE
		case TURNLEFT:
			if (abs(encRight.ticks.now - cmdNavigationValues.encRightOld) < degreeTicks_) {	//@OPTIMIZE
				//---- Settings results (left back, right forw)
				leftPIDval.Setpoint = 100;		//l_pwm
				rightPIDval.Setpoint = 100;		//r_pwm	
				setDriverDirection(4);			// left Backward, right Forward
			}
			else {
				statusArduino = OK;
				cmdNavigation = NOCMD;	//@IO?
				// Reset
				degreeTicks_ = 0;
				leftPID.SetITerm(0);
				rightPID.SetITerm(0);
				printInformationFlag = 1;
				//@TESTING
				leftPIDval.Setpoint = 0;		//l_pwm
				rightPIDval.Setpoint = 0;		//r_pwm
			}
			break;
		case TURNLEFT90:break;		// fix 90° turn right
		case TURNRIGHT: 
			if (abs(encLeft.ticks.now - cmdNavigationValues.encLeftOld) < degreeTicks_) {	//@OPTIMIZE
				leftPIDval.Setpoint = 100;		//l_pwm
				rightPIDval.Setpoint = 100;		//r_pwm	
				setDriverDirection(3);			// left Forward, right Backward		//@OPTIMIZE only difference
			}
			else {
				statusArduino = OK;
				cmdNavigation = NOCMD;	//@IO?
				// Reset
				degreeTicks_ = 0;
				leftPID.SetITerm(0);
				rightPID.SetITerm(0);
				printInformationFlag = 1;
				//@TESTING
				leftPIDval.Setpoint = 0;		//l_pwm
				rightPIDval.Setpoint = 0;		//r_pwm	
			}
			break;
		case TURNRIGHT90: break;		// fix 90° turn right
		default:
			statusArduino = OK;
		break;
		}
		if (printInformationFlag == 1) {
			printInformation(statusArduino, error);
		}
	}
	else {

	}

}


/*---------- || < Functions - Init > || ----------*/
/*------- Driver ------*/
void initDriver() {
	pinMode(pwmDriverIn1, OUTPUT);
	pinMode(pwmDriverIn2, OUTPUT);
	pinMode(pwmDriverIn3, OUTPUT);
	pinMode(pwmDriverIn4, OUTPUT);
	driverLeft.direction = STOP;
	driverRight.direction = STOP;
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

/*------- LED ------*/
void initMainboardGPIO() {
	//----- Taster
	pinMode(tasterRight, INPUT);
	//----- rgbLeft
	rgbLeft.red = rgbLeftRed; rgbLeft.green = rgbLeftGreen; rgbLeft.blue = rgbLeftBlue;
	pinMode(rgbLeft.red, OUTPUT); pinMode(rgbLeft.green, OUTPUT); pinMode(rgbLeft.blue, OUTPUT);
	rgbSet(rgbLeft, BLANK);
	//----- rgbRight
	rgbRight.red = rgbRightRed; rgbRight.green = rgbRightGreen; rgbRight.blue = rgbRightBlue;
	pinMode(rgbRight.red, OUTPUT); pinMode(rgbRight.green, OUTPUT); pinMode(rgbRight.blue, OUTPUT);
	rgbSet(rgbRight, BLANK);
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


/*---------- || Functions < LED > || ----------*/
/**
 * rgbSet(rgb, color)
 * rgb:		rgb_t ...
 * color:	BLANK, RED, GREEN, BLUE
*/
void rgbSet(rgb_t rgb, color_ color) {
	switch (color) {
	case BLANK: digitalWrite(rgb.red, HIGH); digitalWrite(rgb.green, HIGH); digitalWrite(rgb.blue, HIGH); break;
	case RED:digitalWrite(rgb.red, LOW); digitalWrite(rgb.green, HIGH); digitalWrite(rgb.blue, HIGH); break;
	case GREEN:digitalWrite(rgb.red, HIGH); digitalWrite(rgb.green, LOW); digitalWrite(rgb.blue, HIGH); break;
	case BLUE: digitalWrite(rgb.red, HIGH); digitalWrite(rgb.green, HIGH); digitalWrite(rgb.blue, LOW); break;
	default:break;
	}
}

/*---------- || Functions < Testing > || ----------*/
void testSerialResponse() {
	Serial.println(statusArduino);	// Send status, enum value
}

/*---------- || < ISR > || ----------*/
void encoderLeft() {
	encoderInterrupt(&encLeft, interruptEncLeftA, interruptEncLeftB);
}

void encoderRight() {
	encoderInterrupt(&encRight, interruptEncRightA, interruptEncRightB);
}
/*---------- || < Casting > || ----------*/
void setDriverDirection(int value) {
	switch (value) {
	case 0:	driverLeft.direction = STOP; driverRight.direction = STOP; break;	//DONT USE
	case 1: driverLeft.direction = FORWARD; driverRight.direction = FORWARD;
		break;
	case 2: driverLeft.direction = BACKWARD; driverRight.direction = BACKWARD;
		break;
	case 3: driverLeft.direction = FORWARD; driverRight.direction = BACKWARD;
			break;
	case 4: driverLeft.direction = BACKWARD; driverRight.direction = FORWARD;
			break;
	default:break;
	} 
}

/*---------- || < Serial > || ----------*/
void printcmdNavigationValues() {
	Serial.print("Direction: "); Serial.println(cmdNavigationValues.direction);
	Serial.print("Left Value: ");Serial.println(cmdNavigationValues.leftValue);
	Serial.print("Left Increment: "); Serial.println(cmdNavigationValues.leftIncrement);
	Serial.print("Left Degree"); Serial.println(cmdNavigationValues.leftDegree);
	Serial.print("Right Value: "); Serial.println(cmdNavigationValues.rightValue);
	Serial.print("Right Increment: "); Serial.println(cmdNavigationValues.rightInrement);
	Serial.print("Right Degree: "); Serial.println(cmdNavigationValues.rightDegree);
}
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
void printPIDintegral() {
	Serial.print("ITerm (R): ");
	Serial.println(rightPID.GetITerm());
	//Serial.print("Output PWM: ");
	//Serial.println(rightPIDval.Output);
}
void printInformation(status status, errorCode error) {
	Serial.print(statusArduino); Serial.print(",");
	Serial.print(error); Serial.print(",");
	Serial.print(encLeft.ticks.now); Serial.print(",");
	Serial.print(encRight.ticks.now);
	Serial.print("\n");
}
/**
 *	Serial - PID Fehlerdiff. Left & Right
*/
void printPIDvalue() {
	//----- Right
	Serial.println("-----RIGHT ----------");
	//--- PID
	Serial.print("Setpoint"); Serial.println(rightPIDval.Setpoint);
	Serial.print("T/s: "); Serial.println(rightPIDval.Input);
	Serial.print("Kp: "); Serial.println(rightPIDval.Kp);
	//--- Difference
	Serial.print("Delta: "); Serial.println(rightPIDval.Input - rightPIDval.Setpoint);
	//----- Left
	Serial.println("-----LEFT -----");
	//--- PID
	Serial.print("Setpoint:"); Serial.println(leftPIDval.Setpoint);
	Serial.print("T/s: "); Serial.println(leftPIDval.Input);
	//--- Difference
	Serial.print("Delta: "); Serial.println(leftPIDval.Input - leftPIDval.Setpoint);
	//--- Difference Right - Left
	Serial.print("-> DELTA MM: "); Serial.println((leftPIDval.Input - leftPIDval.Setpoint)
		- (rightPIDval.Input - rightPIDval.Setpoint));//Left - Right$
}


/*---------- || < Kommunikation > || ----------*/
void serialEvent() {
	if (Serial.available()) {
		cmdRaspb = Serial.read();	// Erstes Zeichen einlesen

		switch (cmdRaspb) {
			// Reset Error
		case 'E':
			s1 = Serial.read();// "\n"
			break;
		case 'I':
			Serial.read();
			break;
		case 'M': //Motor ; MRightv,Leftv,Dir
			s1 = Serial.readStringUntil(',');
			Serial.read();//","
			s2 = Serial.readStringUntil(',');
			Serial.read();//","
			s3 = Serial.readStringUntil('\n');	//"\n"
			Serial.read();//"\n"
			if (s1 != NULL && s2 != NULL && s3 != NULL) {
				cmdNavigationValues.leftValue = s1.toInt();		//l_pwm
				cmdNavigationValues.rightValue = s2.toInt();		//r_pwm
				cmdNavigationValues.direction = s3.toInt();	//dir
			}
			else {
				error = SYNTAXERROR;
				Serial.readString();
			}
			break;

		case 'S': //Stop
			Serial.read();
			cmdNavigationValues.rightValue = 0;		//r_pwm
			cmdNavigationValues.leftValue = 0;		//l_pwm
			break;

		case 'C': //move n steps
			s1 = Serial.readStringUntil(',');
			Serial.read();//","
			s2 = Serial.readStringUntil(',');
			Serial.read();//","
			s3 = Serial.readStringUntil('\n');	//"\n"
			Serial.read();//"\n"
			if (s1 != NULL && s2 != NULL && s3 != NULL) {
				cmdNavigationValues.leftIncrement = s1.toInt();	//l_incr
				cmdNavigationValues.rightInrement = s2.toInt();	//r_incr
				cmdNavigationValues.direction = s3.toInt(); //dir
			}
			else {
				error = SYNTAXERROR;
				Serial.readString();
			}
			break;

		case 'L': //LeftDegree: L degree
			s1 = Serial.readStringUntil('\n');
			Serial.read();
			if (s1 != NULL) {
				cmdNavigationValues.leftDegree = s1.toInt();	//l_deg
			}
			else {
				error = SYNTAXERROR;
				Serial.readString();
			}
			break;

		case 'R': //RightDegree: R degree
			s1 = Serial.readStringUntil('\n');
			Serial.read();
			if (s1 != NULL) {
				cmdNavigationValues.rightDegree = s1.toInt(); //r_deg
			}
			else {
				error = SYNTAXERROR;
				Serial.readString();
			}
			break;

		default:
			error = ARDCOMMUNICATION;
			//Serial.print(Serial.readString()); //Testing
			break;
		}
	}
}
