/*
 * @file    	Motor.h
 * @version 	v10.0
 * @date    	01.01.20xx
 * @changed 	07.03.2019
 * @author  	cveigl, mzahedi
 * @brief   	DESCRIPTION
 */

#ifndef MOTOR_H_
#define MOTOR_H_

/****************************************************************** INCLUDES */

#include <stdint.h>
#include "Config.h"

/******************************************************************* EXTERNS */

extern class Motor motors; // FIXME: change this

/********************************************************** CLASS DEFINITION */

class Motor {
public:
	static const uint8_t MOTORS_MAX_CURRENT = 5; // [A]

	Motor(void);
	~Motor(void);

	void init(void);
#if MOTOR_CTR_TEST
	void test(void);
#endif
	void softStop(void);
	void hardStop(void);
	void setIntendedParam(int left, int right, int duration);
	void performIntendedMovement(void);

protected:
	int handleObstacles();
	int intendedRightSpeed;
	int intendedLeftSpeed;
	int intendedDuration;
	int rightSpeed;
	int leftSpeed;

private:
	const uint8_t STEPLENGTH = 50;
	const uint8_t STEPLENGTH_TURN = 20;
	const uint8_t ACCEL_STEP = 2;
	const uint8_t MOVE_THRESHOLD = ACCEL_STEP * 2;
};

#endif /* MOTOR_H_ */
