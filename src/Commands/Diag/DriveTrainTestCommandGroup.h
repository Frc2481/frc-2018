/*
 * DriveTrainTestCommandGroup.h
 *
 *  Created on: Jan 15, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DIAG_DRIVETRAINTESTCOMMANDGROUP_H_
#define SRC_COMMANDS_DIAG_DRIVETRAINTESTCOMMANDGROUP_H_l

#include "CommandBase.h"
#include "DriveTrainRunDriveTestCommand.h"
#include "DriveTrainCheckCommand.h"
#include "../DriveTrainStopCommand.h"

class DriveTrainTestCommandGroup : public CommandGroup {
public:
	DriveTrainTestCommandGroup() : CommandGroup("DriveTrainTestCommandGroup") {
		AddParallel(new DriveTrainRunDriveTestCommand());
		AddSequential(new WaitCommand(1.5));
		AddSequential(new DriveTrainCheckCommand());
		AddSequential(new DriveTrainStopCommand());
	}
};

#endif /* SRC_COMMANDS_DIAG_DRIVETRAINTESTCOMMANDGROUP_H_ */
