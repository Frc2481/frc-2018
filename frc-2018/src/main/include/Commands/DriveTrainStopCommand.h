/*
 * DriveTrainStopCommand.h
 *
 *  Created on: Jan 15, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINSTOPCOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINSTOPCOMMAND_H_

#include "CommandBase.h"

class DriveTrainStopCommand : public frc::InstantCommand {
public:
	DriveTrainStopCommand() : InstantCommand("Drive Train Stop Command") {
		Requires(CommandBase::m_driveTrain.get());
	}

	void Initialize() {
		CommandBase::m_driveTrain->Stop();
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINSTOPCOMMAND_H_ */
