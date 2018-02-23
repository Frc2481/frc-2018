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

class AutoCubeCommandGroup : public CommandGroup{
public:
	AutoCubeCommandGroup(std::string path) : CommandGroup("AutoCubeCommandGroup") {
		AddSequential(new ArmToIntakeBack(""));
		AddParallel(new IntakeAcquireCubeCommandGroup());
		AddSequential(new DriveTrainFollowPath(path));
	}
};

#endif /* SRC_COMMANDS_AUTOCUBECOMMANDGROUP_H_ */
