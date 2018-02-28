/*
 * AutoCubeCommandGroup.h
 *
 *  Created on: Feb 19, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_AUTOCUBECOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOCUBECOMMANDGROUP_H_

#include "Commands/CommandGroup.h"
#include "Commands/IntakeAcquireCubeCommandGroup.h"
#include "Commands/ObserverResetPosCommand.h"
#include "Commands/DriveTrainWaitForFieldXorYCommandGroup.h"
#include "Commands/ArmBaseCommand.h"
#include "Commands/DriveTrainFollowPath.h"
#include "Commands/DriveTrainStopCommand.h"

class AutoCubeCommandGroup : public CommandGroup {
public:
	AutoCubeCommandGroup(std::string path, int x, int y) : CommandGroup("AutoCubeCommandGroup") {
		AddParallel(new DriveTrainFollowPath(path));
		if(x != -1 || y != -1) {
//			AddSequential(new WaitCommand(.5));
			AddParallel(new ArmToSwitchBack(""));
		}

//		AddParallel(new IntakeAcquireCubeCommandGroup());

		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));

		AddSequential(new DriveTrainWaitForFieldXorYCommandGroup(x, y));
		AddSequential(new ArmToIntakeBack(""));

		AddSequential(new IntakeHasCubeCommand());
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new DriveTrainStopCommand());
	}
};

#endif /* SRC_COMMANDS_AUTOCUBECOMMANDGROUP_H_ */
