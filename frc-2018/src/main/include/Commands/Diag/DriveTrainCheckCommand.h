/*
 * DriveTrainCheckCommand.h
 *
 *  Created on: Jan 15, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DIAG_DRIVETRAINCHECKCOMMAND_H_
#define SRC_COMMANDS_DIAG_DRIVETRAINCHECKCOMMAND_H_

#include "CommandBase.h"
#include "Subsystems/DriveTrain.h"

class DriveTrainCheckCommand : public frc::InstantCommand {
public:
	DriveTrainCheckCommand() : InstantCommand("DriveTrainCheckCommand") {

	}

	void Initialize() {
		CommandBase::m_driveTrain->CheckDiagnostics();

	}
};

#endif /* SRC_COMMANDS_DIAG_DRIVETRAINCHECKCOMMAND_H_ */
