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

/********************************************************** CLASS DEFINITION */

class Odometry {
public:
  Odometry();
  ~Odometry();

  void init();
  void getStateRightMotor(void);
  void getStateLeftMotor(void);

private:
  int riIncState;
  int riLastIncState;    
  int riDecState;
  int riLastDecState;
  int leIncState;
  int leLastIncState;
  int leDecState;
  int leLastDecState;
  int counterRe;
  int counterLe;
};

#endif // ODOMETRY_H

