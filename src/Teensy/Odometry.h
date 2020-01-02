/*
 * @file    Odometry.h
 * @version v1.0
 * @date    19.11.2018
 * @author  mzahedi
 * @brief   DESCRIPTION
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

/****************************************************************** INCLUDES */

#include "support.h"

#define MOTOR_STOPPED 1000   // odometry timing code for stopped motor
#define MILLIMETER_PER_ENCODER_STEP 511 

void getStateRightMotor(void);
void getStateLeftMotor(void);

/********************************************************** CLASS DEFINITION */

extern class Odometry odo;

class Odometry {
public:
  Odometry();
  ~Odometry();

  void init();
  void printEncoderValues();
  void clearEncoderValues();
  int32_t getRightEncoderValue();
  int32_t getLeftEncoderValue();
  uint16_t getRightEncoderTime();
  uint16_t getLeftEncoderTime();
  int8_t getRightDirection();
  int8_t getLeftDirection();

  void updateOdometry();
  void printOdom();

  double get_odom_right();
  double get_odom_left();
};

#endif // ODOMETRY_H
