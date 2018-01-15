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
	static constexpr int k_ticksPerRev = 128;
	static constexpr double k_inchesPerRev = 3.25 * 3.14159265;
	static constexpr double k_robotLength = 22; //actually come back and change
	static constexpr double k_robotWidth = 25; //same
	static constexpr double k_encoderRevPerWheelRev = 15.47;
	static constexpr double k_speedP = 1;
	static constexpr double k_speedI = 0.0005;
	static constexpr double k_speedD = 10;
	static constexpr double k_steerP = 3;
	static constexpr double k_steerI = 0;
	static constexpr double k_steerD = 40;

};

#endif /* SRC_COMPONENTS_SWERVEMODULEV2CONSTANTS_H_ */
