/*
 * SwerveModuleV2Constants.h
 *
 *  Created on: Aug 7, 2017
 *      Author: Team2481
 */

#ifndef SRC_COMPONENTS_SWERVEMODULEV2CONSTANTS_H_
#define SRC_COMPONENTS_SWERVEMODULEV2CONSTANTS_H_

class SwerveModuleV2Constants {
public:
	SwerveModuleV2Constants();
	virtual ~SwerveModuleV2Constants();
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

};

#endif /* SRC_COMPONENTS_SWERVEMODULEV2CONSTANTS_H_ */
