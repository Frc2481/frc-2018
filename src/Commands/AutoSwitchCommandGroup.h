/*
 * AutoSwitchCommandGroup.h
 *
 *  Created on: Feb 19, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_AUTOSWITCHCOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOSWITCHCOMMANDGROUP_H_

#include "CommandBase.h"
#include "Commands/CommandGroup.h"
#include "Commands/IntakeReleaseCubeCommandGroup.h"
#include "Commands/ObserverResetPosCommand.h"
#include "Commands/ArmExtentionMotionScaling.h"
#include "Commands/ArmBaseCommand.h"
#include "Commands/DriveTrainFollowPath.h"
#include "Commands/ArmWaitForPivotOnTargetCommand.h"
#include "DriveTrainDriveCommand.h"

class AutoSwitchCommandGroup : public CommandGroup{
public:
	AutoSwitchCommandGroup(std::string path, bool isFrontIntakeRelease, int xRelease = -1, int yRelease = -1) : CommandGroup("AutoSwitchCommandGroup"){
//		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("StartSwitch"));
		if(isFrontIntakeRelease) {
			AddSequential(new ArmToSwitchFront(""), 1.0);
		}
		else {
			AddSequential(new ArmToSwitchBack(""), 1.0);
		}
		if(path.empty()) {
			AddSequential(new DriveTrainDriveCommand(0, -1, 0, 0.5));
		}
		else {
			AddParallel(new DriveTrainFollowPath(path));
			AddSequential(new DriveTrainWaitForFieldXorYCommandGroup(xRelease, yRelease));
		}

		AddSequential(new WaitCommand(0.5));
		AddSequential(new ArmWaitForPivotOnTargetCommand());
		AddSequential(new IntakeReleaseCubeCommandGroup(0.7));
	}

	void Initialize() {
		printf("Command Switch\n");
	}

};

#endif /* SRC_COMMANDS_AUTOSWITCHCOMMANDGROUP_H_ */
