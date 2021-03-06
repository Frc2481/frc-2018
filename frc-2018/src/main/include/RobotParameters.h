/*
 * RobotParameters.h
 *
 *  Created on: Jan 20, 2017
 *      Author: FIRSTMentor
 */
#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H
//#include "Components/PersistedSettings.h"
#include <math.h>
#define DEBUGGING 1
#define ROBOT_SETTING_PATH "settings.cfg"

class RobotParameters {
public:
	// wheel drive encoder
	static constexpr int k_ticksPerEncoderRev = 512; // 128 * 4
	static constexpr double k_encoderRevPerWheelRevLowGear = 6.7403;
	static constexpr double k_encoderRevPerWheelRevHighGear = 3.1166;

	//low: 12:36 * 34:50 * 34:50 * 80:32 * 16:44 = .1401
	//high: 12:36 * 34:50 * 50:34 * 80:32 * 16:44 = .303


	// Wheel to bevel:          44:16 was 48:30
	// Big Sprk to Little Sprk: 58:17
	// Shaft to Encoder:        36:12

	//(44/16) * (58/17) * (36/12)


	static constexpr double k_encoderTicksPerPivotDegree = 46.65;

	//chain old 60:22
	//OLD-> 60:22 * 4096 / 360 = 31.03

	//chain 82:20
	//82:20 * 4096 / 360 = 46.65

	static constexpr double k_encoderTicksPerExtensionInch = 577.95;

	//belt 1 / 7.087
	//4086 / (1 / 7.087)

	static constexpr double k_inchesPerWheelRev = 3.95 * 3.14159265; // pi * diameter
	static constexpr double k_robotLength = 27.0;
	static constexpr double k_robotWidth = 22.375;
	static constexpr double k_speedP = 1;
	static constexpr double k_speedI = 0.0005;
	static constexpr double k_speedD = 10;
	static constexpr double k_steerP = 3;
	static constexpr double k_steerI = 0;
	static constexpr double k_steerD = 40;

	static constexpr double kpPos = 0.02; //0.01; //0.007;
	static constexpr double kiPos = 0;
	static constexpr double kdPos = 0;
	static constexpr double kfPos = 0; //0.007;
	static constexpr double kIZonePos = 0;

	static constexpr double kpYaw = 0.02;
	static constexpr double kiYaw = 0.0;
	static constexpr double kdYaw = 0;
	static constexpr double kfYaw = 0;
	static constexpr double kIZoneYaw = 5.0;

	static constexpr double PositionControllerPeriod = 0.05;

	static constexpr double kTolerancePos = 3.0; //3.0;
	static constexpr double kToleranceHeading = 3.0; //3.0;

	static constexpr double k_extenderUpP = 0.3; //ToDo: change values //40% * 1023 / 3700
	static constexpr double k_extenderDownP = 0.5;
	static constexpr double k_extenderI = 0;
	static constexpr double k_extenderD = 0;
	static constexpr double k_extenderF = 0.445;
	static constexpr double k_extenderVelocityUp = 4000; //2600;
	static constexpr double k_extenderVelocityDown = 4000;
	static constexpr double k_extenderAccelerationUp = 16000; //2300;
	static constexpr double k_extenderAccelerationDown = 16000;
	static constexpr double k_extenderPeakOutputForward = 1.0;
	static constexpr double k_extenderPeakOutputReverse = -1.0;

	static constexpr double k_pivotP = 2.5;
	static constexpr double k_pivotI = 0;
	static constexpr double k_pivotD = 100;
	static constexpr double k_pivotF = 0.639;

	static constexpr double k_pivotVelocity = 1600;
	static constexpr double k_pivotAcceleration = 5000;
	static constexpr double k_pivotDecceleration = 2500;
	static constexpr double k_pivotPeakOutputForward = 1.0;
	static constexpr double k_pivotPeakOutputReverse = -1.0;
	static constexpr double k_pivotVelocityFudge = 300;

	static constexpr double k_minRobotExtend = 32.25;
	static constexpr double k_gripperThickness = 1;
	static constexpr double k_farthestPointLimit = 32.375;
	static constexpr double k_pivotToMidpointPOB = 0.83;
	static constexpr double k_pivotToMidpointPOT = 1.83;
	static constexpr double k_upperLeftBound = -24;
	static constexpr double k_upperRightBound = 26;
	static constexpr double k_lowerLeftBound = -123;
	static constexpr double k_lowerRightBound = 127;
	static constexpr double k_right90 = 90;
	static constexpr double k_left90 = -90;
	static constexpr double k_rightFrameConstrained = 58; //calculated: 49, changed to be optimal
	static constexpr double k_leftFrameConstrained = -66; //-55
  
	static constexpr double cameraOffsetX = 5;
	static constexpr double cameraOffsetY = 4;
	static constexpr double cameraOffsetZ = 28.375;
	static constexpr double cameraOffsetRoll = 0;
	static constexpr double cameraOffsetPitch = -19;
	static constexpr double cameraOffsetYaw = 0;
	static constexpr double cubeHeight = 11.0;
};
#endif
