/*
 * DriveTrainSetBrakeCommand.h
 *
 *  Created on: Apr 21, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINSETBRAKECOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINSETBRAKECOMMAND_H_

#include "CommandBase.h"

class DriveTrainSetBrakeCommand : public InstantCommand {
private:
	bool m_isBraked;
public:
	DriveTrainSetBrakeCommand(bool isBraked) : InstantCommand("DriveTrainSetBrakeCommand") {
		m_isBraked = isBraked;
	}
	virtual ~DriveTrainSetBrakeCommand(){}

	void Initialize() {
		CommandBase::m_driveTrain->SetBrake(m_isBraked);
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINSETBRAKECOMMAND_H_ */
