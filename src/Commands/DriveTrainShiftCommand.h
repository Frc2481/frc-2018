/*
 * DriveTrainShiftCommand.h
 *
 *  Created on: Jan 15, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINSHIFTCOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINSHIFTCOMMAND_H_

#include "../CommandBase.h"

class DriveTrainShiftCommand : public CommandBase {
public:
	DriveTrainShiftCommand() : CommandBase("DriveTrainShiftCommand") {
	}

	void Initialize() {
		m_driveTrain->Shift(true);
	}

	bool IsFinished() {
		return false;
	}

	void Interrupted() {
		m_driveTrain->Shift(false);
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINSHIFTCOMMAND_H_ */
