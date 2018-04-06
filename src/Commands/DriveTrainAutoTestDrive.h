/*
 * DriveTrainAutoTestDrive.h
 *
 *  Created on: Apr 2, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINAUTOTESTDRIVE_H_
#define SRC_COMMANDS_DRIVETRAINAUTOTESTDRIVE_H_

#include "CommandBase.h"

class DriveTrainAutoTestDrive : public CommandBase{
public:
	DriveTrainAutoTestDrive() : CommandBase("DriveTrainAutoTestDrive"){
		Requires(m_driveTrain.get());
	}
	virtual ~DriveTrainAutoTestDrive() {}

	void Initialize() {}
	void Execute() {
		m_driveTrain->Drive(0, TimeSinceInitialized() * 1 / 48.0, 0);
	}
	bool IsFinished() {
		return TimeSinceInitialized() > 56;
	}
	void End() {
		m_driveTrain->Drive(0, 0, 0);
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINAUTOTESTDRIVE_H_ */
