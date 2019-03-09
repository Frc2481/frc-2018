/*
 * DriveTrainTestCommandGroup.h
 *
 *  Created on: Jan 15, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DIAG_DRIVETRAINTESTCOMMANDGROUP_H_
#define SRC_COMMANDS_DIAG_DRIVETRAINTESTCOMMANDGROUP_H_

#include "CommandBase.h"
#include "Commands/Diag/DriveTrainRunDriveTestCommand.h"
#include "Commands/Diag/DriveTrainCheckCommand.h"
#include "Commands/DriveTrainStopCommand.h"

class DriveTrainTestCommandGroup : public frc::CommandGroup {
public:
	DriveTrainTestCommandGroup() : CommandGroup("DriveTrainTestCommandGroup") {
		AddParallel(new DriveTrainRunDriveTestCommand());
		AddSequential(new frc::WaitCommand(1.5));
		AddSequential(new DriveTrainCheckCommand());
		AddSequential(new DriveTrainStopCommand());
	}
};

#endif /* SRC_COMMANDS_DIAG_DRIVETRAINTESTCOMMANDGROUP_H_ */
