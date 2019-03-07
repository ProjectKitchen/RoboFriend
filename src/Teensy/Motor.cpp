/*
 * @file    	Motor.cpp
 * @version 	v10.0
 * @date    	01.01.20xx
 * @changed 	07.03.2019
 * @author  	cveigl, mzahedi
 * @brief   	DESCRIPTION
 */

/****************************************************************** INCLUDES */

#include <Arduino.h>
#include <TimerThree.h>
#include "Motor.h"
#include "Config.h"
#include "GPIO.h"
#include "Sensor.h"

/******************************************************************* GLOBALS */

#if ROBOFRIEND_VERSION == 1
int LEFT_MOTOR_MINPWM = 20;
int LEFT_MOTOR_MAXPWM = 512;
int RIGHT_MOTOR_MINPWM = 20;
int RIGHT_MOTOR_MAXPWM = 600;
#elif ROBOFRIEND_VERSION == 2
int LEFT_MOTOR_MINPWM = 20;
int LEFT_MOTOR_MAXPWM = 512;
int RIGHT_MOTOR_MINPWM = 20;
int RIGHT_MOTOR_MAXPWM = 1500;	// actually a big difference for the right motor! a motor speed control loop might be mandatory !
#else
#error "You can only choose between Robofriend v1 and v2!"
#endif

/******************************************************************* EXTERNS */

extern long loopcounter; // FIXME: change this

/*************************************************************** DEFINITIONS */

Motor::Motor() {

}

Motor::~Motor() {

}

void Motor::init() {
	// switch the open drain transistor output - see the schematics
	pinMode(PIN_OC, OUTPUT);
	digitalWrite(PIN_OC, HIGH);

	Timer3.initialize(100);   // 100us period = 10kHz frequency for PWM pins

	pinMode(PIN_MT_RI_FWD, OUTPUT);
	pinMode(PIN_MT_RI_BCK, OUTPUT);
	pinMode(PIN_MT_RI_PWM, OUTPUT);

	pinMode(PIN_MT_LE_FWD, OUTPUT);
	pinMode(PIN_MT_LE_BCK, OUTPUT);
	pinMode(PIN_MT_LE_PWM, OUTPUT);

	softStop();
}

#if MOTOR_CTR_TEST
void Motor::test() {
	/* move forward */
	digitalWrite(PIN_MT_LE_FWD, HIGH);
	digitalWrite(PIN_MT_RI_FWD, HIGH);
	digitalWrite(PIN_MT_LE_BCK, LOW);
	digitalWrite(PIN_MT_RI_BCK, LOW);
	Timer3.pwm(PIN_MT_RI_PWM, 1023);
	Timer3.pwm(PIN_MT_LE_PWM, 1023);
	delay(5000);

	/* turn left */
	digitalWrite(PIN_MT_LE_FWD, LOW);
	digitalWrite(PIN_MT_RI_FWD, HIGH);
	digitalWrite(PIN_MT_LE_BCK, LOW);
	digitalWrite(PIN_MT_RI_BCK, LOW);
	delay(5000);

	/* turn right */
	digitalWrite(PIN_MT_LE_FWD, HIGH);
	digitalWrite(PIN_MT_RI_FWD, LOW);
	digitalWrite(PIN_MT_LE_BCK, LOW);
	digitalWrite(PIN_MT_RI_BCK, LOW);
	delay(5000);

	/* move backward */
	digitalWrite(PIN_MT_LE_FWD, LOW);
	digitalWrite(PIN_MT_RI_FWD, LOW);
	digitalWrite(PIN_MT_LE_BCK, HIGH);
	digitalWrite(PIN_MT_RI_BCK, HIGH);
	delay(5000);

	/* reset pins */
	softStop();
}
#endif

void Motor::softStop() {
	Timer3.pwm(PIN_MT_LE_PWM, 0);
	Timer3.pwm(PIN_MT_RI_PWM, 0);

	digitalWrite(PIN_MT_LE_FWD, LOW);
	digitalWrite(PIN_MT_RI_FWD, LOW);
	digitalWrite(PIN_MT_LE_BCK, LOW);
	digitalWrite(PIN_MT_RI_BCK, LOW);

	intendedLeftSpeed = leftSpeed = 0;
	intendedRightSpeed = rightSpeed = 0;
	intendedDuration = 0;
}

void Motor::hardStop() {
	digitalWrite(PIN_OC, LOW);

	intendedLeftSpeed = leftSpeed = 0;
	intendedRightSpeed = rightSpeed = 0;
	intendedDuration = 0;
}

void Motor::setIntendedParam(int left, int right, int duration) {
	Serial.printf("Drive right=%04d, left=%04d, duration=%04d\n", left, right, duration);
	intendedRightSpeed = right;
	intendedLeftSpeed = left;
	intendedDuration = duration;
}

void Motor::performIntendedMovement() {
	bool leftSpeedReached = false, rightSpeedReached = false;

	if (intendedLeftSpeed >= leftSpeed + ACCEL_STEP)
		leftSpeed += ACCEL_STEP;
	else if (intendedLeftSpeed <= leftSpeed - ACCEL_STEP)
		leftSpeed -= ACCEL_STEP;
	else
		leftSpeedReached = true;

	if (intendedRightSpeed >= rightSpeed + ACCEL_STEP)
		rightSpeed += ACCEL_STEP;
	else if (intendedRightSpeed <= rightSpeed - ACCEL_STEP)
		rightSpeed -= ACCEL_STEP;
	else
		rightSpeedReached = true;

	if (leftSpeedReached && rightSpeedReached && (intendedDuration > 0)) {
		intendedDuration--;
		if (!intendedDuration) {
			intendedLeftSpeed = 0;
			intendedRightSpeed = 0;
		}
	}

#if HANDLE_OBSTACLES
	handleObstacles();
#endif

	if (leftSpeed == 0) {
		digitalWrite(PIN_MT_LE_FWD, LOW);
		digitalWrite(PIN_MT_LE_BCK, LOW);
	} else if (leftSpeed > MOVE_THRESHOLD) {
		digitalWrite(PIN_MT_LE_BCK, LOW);
		digitalWrite(PIN_MT_LE_FWD, HIGH);
	} else if (leftSpeed < -MOVE_THRESHOLD) {
		digitalWrite(PIN_MT_LE_FWD, LOW);
		digitalWrite(PIN_MT_LE_BCK, HIGH);
	}

	if (rightSpeed == 0) {
		digitalWrite(PIN_MT_RI_FWD, LOW);
		digitalWrite(PIN_MT_RI_FWD, LOW);
	} else if (rightSpeed > MOVE_THRESHOLD) {
		digitalWrite(PIN_MT_RI_BCK, LOW);
		digitalWrite(PIN_MT_RI_FWD, HIGH);
	} else if (rightSpeed < -MOVE_THRESHOLD) {
		digitalWrite(PIN_MT_RI_FWD, LOW);
		digitalWrite(PIN_MT_RI_BCK, HIGH);
	}

	if (rightSpeed) {
		Timer3.pwm(PIN_MT_RI_PWM,
				map((rightSpeed < 0) ? -rightSpeed : rightSpeed, 0, 255,
						LEFT_MOTOR_MINPWM, RIGHT_MOTOR_MAXPWM)); // right motor slighty slower: compensate PWMs!
	} else
		Timer3.pwm(PIN_MT_RI_PWM, 0);

	if (leftSpeed)
		Timer3.pwm(PIN_MT_LE_PWM,
				map((leftSpeed < 0) ? -leftSpeed : leftSpeed, 0, 255,
						LEFT_MOTOR_MINPWM, LEFT_MOTOR_MAXPWM));
	else
		Timer3.pwm(PIN_MT_LE_PWM, 0);

#if DUMP_MOTOR_MSG
	if (((loopcounter % 3) == 0) && (leftSpeed || rightSpeed))
		Serial.printf("Speed: %d/%d\n", rightSpeed, leftSpeed);
#endif
}

int Motor::handleObstacles() {
	int triggered = 0;

	if (sensors.isIRSensorRightTriggered()) {
		triggered = 1;
		if (rightSpeed > 0)
			rightSpeed = 0;
#if DUMP_SENSOR_MSG
		if ((loopcounter % 10) == 0)
			Serial.printf("Right %04d ", sensors.getIRSensorRightValue());
#endif
	}
	if (sensors.isIRSensorLeftTriggered()) {
		triggered = 1;
		if (leftSpeed > 0)
			leftSpeed = 0;
#if DUMP_SENSOR_MSG
		if ((loopcounter % 10) == 0)
			Serial.printf("Left %04d ", sensors.getIRSensorLeftValue());
#endif
	}
	if (sensors.isIRSensorMiddleTriggered()) {
		triggered = 1;
		if (rightSpeed > 0)
			rightSpeed = 0;
		if (leftSpeed > 0)
			leftSpeed = 0;
#if DUMP_SENSOR_MSG
		if ((loopcounter % 10) == 0)
			Serial.printf("Middle %04d ", sensors.getIRSensorMiddleValue());
#endif
	}
#if DUMP_SENSOR_MSG
	if (((loopcounter % 10) == 0) && triggered)
		Serial.println("");
#endif

	return (triggered);
}
