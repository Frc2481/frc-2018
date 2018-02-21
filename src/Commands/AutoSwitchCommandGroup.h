/*
 * AutoSwitchCommandGroup.h
 *
 *  Created on: Feb 19, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_AUTOSWITCHCOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOSWITCHCOMMANDGROUP_H_

#include "CommandBase.h"
#include "Commands/IntakeReleaseCubeCommandGroup.h"
#include "Commands/ObserverResetPosCommand.h"
#include "Commands/ArmExtentionMotionScaling.h"
#include "Commands/ArmBaseCommand.h"
#include "Commands/DriveTrainFollowPath.h"

class AutoSwitchCommandGroup : public CommandGroup{
public:
	AutoSwitchCommandGroup(std::string path, bool isFrontIntakeRelease) : CommandGroup("AutoSwitchCommandGroup"){
		AddSequential(new ArmExtentionMotionScaling(0.3));
		if(isFrontIntakeRelease) {
			AddParallel(new ArmToSwitchFront(""));
		}
		else {
			AddParallel(new ArmToSwitchBack(""));
		}

		AddSequential(new DriveTrainFollowPath(path));
		AddSequential(new IntakeReleaseCubeCommandGroup(0.7));
		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new ArmToStow(""));
	}
};

#endif /* SRC_COMMANDS_AUTOSWITCHCOMMANDGROUP_H_ */
