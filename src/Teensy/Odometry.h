/*
 * @file    	Odometry.h
 * @version 	v10.0
 * @date    	19.11.2018
 * @changed 	07.03.2019
 * @author  	cveigl, mzahedi
 * @brief   	DESCRIPTION
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

/********************************************************** CLASS DEFINITION */

class Odometry {
public:
  Odometry(void);
  ~Odometry(void);

  void init(void);
  void printEncoderValues(void);
  void clearEncoderValues(void);
  int32_t getRightEncoderValue(void);
  int32_t getLeftEncoderValue(void);
};

void readEncoderValues(void);
void readEncoderValueRightMotor(void);
void readEncoderValueLeftMotor(void);

#endif // ODOMETRY_H
