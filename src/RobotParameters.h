/*
 * RobotParameters.h
 *
 *  Created on: Jan 20, 2017
 *      Author: FIRSTMentor
 */
#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H
//#include "Components/PersistedSettings.h"
#include "math.h"
#define DEBUGGING 1
#define ROBOT_SETTING_PATH "settings.cfg"
#define ENCODER_OFFSET_FILENAME "/home/lvuser/Encoder_Offsets"
#define ENCODER_ROTATION_PER_DEGREE_FILENAME "/home/lvuser/Rotations_Per_Degree"

//Robot Base
	#define ROBOT_LENGTH 22
	#define ROBOT_WIDTH 25

#define INCHES_PER_REV 3.25 * 3.14159265 //inches
#define WHEEL_DIAMETER_INCHES 3.25
#define ENCODER_TICKS_PER_REV 21.75 * 128
#define MM_PER_REV 25.4 * INCHES_PER_REV
//DRIVE_FIRST_STAGE_GEAR_RATIO 17.0/58.0 REAL      PRACTICE:18/58
//DRIVE_SECOND_STAGE_GEAR_RATIO 1.0/1.6
//DRIVE_THIRD_STAGE_GEAR_RATIO 36.0/12.0
#define ENCODER_REV_PER_WHEEL_REV  /*16.37//REAL*/   /*PRACTICE:*/15.47
							/*1 / (DRIVE_FIRST_STAGE_GEAR_RATIO * \
								DRIVE_SECOND_STAGE_GEAR_RATIO * \
								DRIVE_THIRD_STAGE_GEAR_RATIO)*/
//#define ENCODER_TICKS_PER_INCH 8725.56317
//#define ENCODER_TICKS_PER_MM 343.526109

#define CIRCUMFERENCE_OF_ROTATION (sqrt((22*22)+(23.5*23.5)))*M_PI

#define GEAR_FLICK_DELAY 1.0
#define HOPPER_SPEED 0.2

#define FEET_PER_SEC_LOW 6.38
#define FEET_PER_SEC_HIGH 13.8
#define ENCODER_UNITS_PER_REV 4096
#define RPM 435
#define FEED_FORWARD_LOW (1 * 1023) / ((FEET_PER_SEC_LOW * 60 * 12 / (INCHES_PER_REV))*(1/60.0)*(1/10.0)*(ENCODER_UNITS_PER_REV / ENCODER_REV_PER_WHEEL_REV))
#define FEED_FORWARD_HIGH (1 * 1023) / ((FEET_PER_SEC_HIGH * 60 * 12 / (INCHES_PER_REV))*(1/60.0)*(1/10.0)*(ENCODER_UNITS_PER_REV / ENCODER_REV_PER_WHEEL_REV))

class RobotParameters {
public:
	static constexpr int k_ticksPerEncoderRev = 512; // 128 * 4
	static constexpr double k_encoderRevPerWheelRev = 28.15; // 2017 value = 15.47

	// Wheel to bevel:          44:16 was 48:30
	// Big Sprk to Little Sprk: 58:17
	// Shaft to Encoder:        36:12

	//(44/16) * (58/17) * (36/12)

	static constexpr double k_inchesPerWheelRev = 4.1 * 3.14159265; // 2017 value = 3.25 // pi * diameter
	static constexpr double k_robotLength = 23.5;
	static constexpr double k_robotWidth = 21.875;
	static constexpr double k_speedP = 1;
	static constexpr double k_speedI = 0.0005;
	static constexpr double k_speedD = 10;
	static constexpr double k_steerP = 3;
	static constexpr double k_steerI = 0;
	static constexpr double k_steerD = 40;

	static constexpr double kpPos = 0.03;
	static constexpr double kiPos = 0.0;
	static constexpr double kdPos = 0;
	static constexpr double kfPos = 0;
	static constexpr double kIZonePos = 5;

	static constexpr double kpYaw = 0.01;
	static constexpr double kiYaw = 0.001;
	static constexpr double kdYaw = 0;
	static constexpr double kfYaw = 1.0; //Does this do anything???
	static constexpr double kIZoneYaw = 5;

	static constexpr double PositionControllerPeriod = 0.05;

	static constexpr double kTolerancePos = 1.0;
	static constexpr double kToleranceHeading = 1.0;

};
#endif
