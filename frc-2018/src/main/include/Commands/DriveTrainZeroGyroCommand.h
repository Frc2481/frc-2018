/*
 * DriveTrainZeroGyroCommand.h
 *
 *  Created on: Feb 20, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_DRIVETRAINZEROGYROCOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINZEROGYROCOMMAND_H_

#include "CommandBase.h"

class DriveTrainZeroGyroCommand : public frc::InstantCommand{
public:
	DriveTrainZeroGyroCommand() : InstantCommand("DriveTrainZeroGyroCommand") {

	}
	virtual ~DriveTrainZeroGyroCommand(){}
	void Initialize() {
		CommandBase::m_driveTrain->ZeroGyro();
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINZEROGYROCOMMAND_H_ */
