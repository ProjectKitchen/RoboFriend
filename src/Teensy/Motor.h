#ifndef MOTOR_H_
#define MOTOR_H_

#define STEPLENGTH      50
#define STEPLENGTH_TURN 20
#define ACCEL_STEP      2
#define MOVE_THRESHOLD  ACCEL_STEP*2

extern class Motor Motors;

class Motor {

public:
  Motor(void);
  ~Motor();

  void init();
  void stop();
  void drive (int left, int right, int duration);
  void updateMotors();
  void drive_odom(int left, int right, int duration);

protected:

  void performPIDControl();
  void performLinearControl();
  void setMotorSpeedGoals(int left, int right);
  
  int intendedRightSpeed;
  int intendedLeftSpeed;
  int intendedDuration;
  int rightSpeed;
  int leftSpeed;
  int handleObstacles();

};

#endif /* MOTOR_H_ */
