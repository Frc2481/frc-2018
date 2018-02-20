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

	static constexpr double k_inchesPerWheelRev = 4.0 * 3.14159265; // pi * diameter
	static constexpr double k_robotLength = 27.0;
	static constexpr double k_robotWidth = 22.25;
	static constexpr double k_speedP = 1;
	static constexpr double k_speedI = 0.0005;
	static constexpr double k_speedD = 10;
	static constexpr double k_steerP = 3;
	static constexpr double k_steerI = 0;
	static constexpr double k_steerD = 40;

	static constexpr double kpPos = 0.03; //0.01; //0.007;
	static constexpr double kiPos = 0;
	static constexpr double kdPos = 0;
	static constexpr double kfPos = 0; //0.007;
	static constexpr double kIZonePos = 0;

	static constexpr double kpYaw = 0.01;
	static constexpr double kiYaw = 0.001;
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
	static constexpr double k_extenderVelocity = 2300;
	static constexpr double k_extenderAcceleration = 2300;
	static constexpr double k_extenderPeakOutputForward = 1.0;
	static constexpr double k_extenderPeakOutputReverse = -1.0;

	static constexpr double k_pivotP = 3;
	static constexpr double k_pivotI = 0;
	static constexpr double k_pivotD = 30;
	static constexpr double k_pivotF = 2.046; //3

	static constexpr double k_pivotVelocity = 900;
	static constexpr double k_pivotAcceleration = 1800;
	static constexpr double k_pivotPeakOutputForward = 1.0;
	static constexpr double k_pivotPeakOutputReverse = -1.0;

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

	static constexpr int k_intake1FrontAngle = 120;
	static constexpr int k_intake2FrontAngle = 104;
	static constexpr int k_intake3FrontAngle = 83;

	static constexpr int k_intake1BackAngle = -119;
	static constexpr int k_intake2BackAngle = -101;
	static constexpr int k_intake3BackAngle = -83;

	static constexpr int k_switchFrontAngle = 91;
	static constexpr int k_switch2FrontAngle = 65;
	static constexpr int k_scaleLowFrontAngle = 39;
	static constexpr int k_scaleLow2FrontAngle = 28;
	static constexpr int k_scaleMidFrontAngle = 25;
	static constexpr int k_scaleMid2FrontAngle = 23;
	static constexpr int k_scaleHighFrontAngle = 17;
	static constexpr int k_scaleHigh2FrontAngle = 16;

	static constexpr int k_switchBackAngle = -86;
	static constexpr int k_switch2BackAngle = -65;
	static constexpr int k_scaleLowBackAngle = -39;
	static constexpr int k_scaleLow2BackAngle = -28;
	static constexpr int k_scaleMidBackAngle = -25;
	static constexpr int k_scaleMid2BackAngle = -23;
	static constexpr int k_scaleHighBackAngle = -17;
	static constexpr int k_scaleHigh2BackAngle = -16;
};
#endif
