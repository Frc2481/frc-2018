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
#include "Commands/ArmWaitForPivotOnTargetCommand.h"

class AutoSwitchCommandGroup : public CommandGroup{
public:
	AutoSwitchCommandGroup(std::string path, bool isFrontIntakeRelease, int xRelease = -1, int yRelease = -1) : CommandGroup("AutoSwitchCommandGroup"){
//		AddSequential(new ArmExtentionMotionScaling(0.3));
		if(isFrontIntakeRelease) {
			AddSequential(new ArmToSwitchFront(""));
		}
		else {
			AddSequential(new ArmToSwitchBack(""));
		}
		AddParallel(new DriveTrainFollowPath(path));


		AddSequential(new DriveTrainWaitForFieldXorYCommandGroup(xRelease, yRelease));
		AddSequential(new WaitCommand(.5));
		AddSequential(new ArmWaitForPivotOnTargetCommand());
		AddSequential(new IntakeReleaseCubeCommandGroup(0.7));
//		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new ArmToStow(""));
	}
};

#endif /* SRC_COMMANDS_AUTOSWITCHCOMMANDGROUP_H_ */
