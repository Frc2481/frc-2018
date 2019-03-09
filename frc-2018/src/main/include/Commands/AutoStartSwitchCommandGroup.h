/*
 * AutoStartSwitchCommandGroup.h
 *
 *  Created on: Mar 2, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOSTARTSWITCHCOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOSTARTSWITCHCOMMANDGROUP_H_

#include "CommandBase.h"
#include "frc/Commands/CommandGroup.h"
#include "Commands/IntakeReleaseCubeCommandGroup.h"
#include "Commands/ObserverResetPosCommand.h"
#include "Commands/ArmExtentionMotionScaling.h"
#include "Commands/ArmBaseCommand.h"
#include "Commands/DriveTrainFollowPath.h"
#include "Commands/ArmWaitForPivotOnTargetCommand.h"
#include "DriveTrainDriveCommand.h"

class AutoStartSwitchCommandGroup : public frc::CommandGroup {
public:
	AutoStartSwitchCommandGroup(std::string path, int xRelease = -1, int yRelease = -1) : CommandGroup("AutoStartSwitchCommandGroup"){
		AddParallel(new ArmToSwitchFront(""), 1.0);
		AddParallel(new DriveTrainFollowPath(path));
		AddSequential(new DriveTrainWaitForFieldXorYCommandGroup(xRelease, yRelease));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerUnloadCommand(0.7));
		AddSequential(new frc::WaitCommand(0.5));
	}
};

#endif /* SRC_COMMANDS_AUTOSTARTSWITCHCOMMANDGROUP_H_ */
