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

void getStateRightMotor(void);
void getStateLeftMotor(void);

/********************************************************** CLASS DEFINITION */

class Odometry {
public:
  Odometry();
  ~Odometry();

  void init();
  void printEncoderValues();
  void clearEncoderValues();
  int32_t getRightEncoderValue();
  int32_t getLeftEncoderValue();
};

#endif // ODOMETRY_H
