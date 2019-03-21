/*
 * @file    	Config.h
 * @version 	v10.0
 * @date    	15.11.2018
 * @changed 	07.03.2019
 * @author  	cveigl, mzahedi
 * @brief   	DESCRIPTION
 */

#ifndef CONFIG_H_
#define CONFIG_H_

/************************************************************ GLOBAL DEFINES */

#define DUMP_MOTOR_MSG 		0
#define DUMP_SENSOR_MSG 	0
#define DUMP_ENCODER_VALUES	0
#define MOTOR_CTR_TEST		0
#define OVERCURRENT_LOGIC	0
#define READ_IR_SENSORS		0

/*********************************************************************** IMU */

#define IMU_ADRESS			0x50

/******************************************************************** MOTORS */

#define HANDLE_OBSTACLES 	0	// currently disables because sensor values are not valid in Robofriend v2!
#define ROBOFRIEND_VERSION  2	// version 1 and version 2 use different motor compensation settings

#endif /* CONFIG_H_ */
