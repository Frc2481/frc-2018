/*
 * DriveControllerOutput.h
 *
 *  Created on: Jan 26, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMPONENTS_DRIVECONTROLLEROUTPUT_H_
#define SRC_COMPONENTS_DRIVECONTROLLEROUTPUT_H_

#include "RobotParameters.h"
#include "frc/WPILib.h"

class DriveControllerOutput : public frc::PIDOutput {
public:
	DriveControllerOutput();
	virtual ~DriveControllerOutput();
	void PIDWrite(double output);
	double GetOutput();

private:
	std::atomic<double> val;

};

#endif /* SRC_COMPONENTS_DRIVECONTROLLEROUTPUT_H_ */
