/*
 * AutoScale1CommandGroup.h
 *
 *  Created on: Feb 8, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOSCALECOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOSCALECOMMANDGROUP_H_

#include "CommandBase.h"
#include "Commands/IntakeReleaseCubeCommandGroup.h"
#include "Commands/ObserverResetPosCommand.h"
#include "Commands/ArmExtentionMotionScaling.h"
#include "Commands/ArmBaseCommand.h"
#include "Commands/DriveTrainFollowPath.h"

class AutoScaleCommandGroup : public CommandGroup{
public:
	AutoScaleCommandGroup(std::string path, std::string pathBackUp) : CommandGroup("AutoScaleCommandGroup"){
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddParallel(new ArmToHighScaleFront(""));
		AddSequential(new DriveTrainFollowPath(path));
		AddSequential(new IntakeReleaseCubeCommandGroup(0.5));
		AddSequential(new DriveTrainFollowPath(pathBackUp));
		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new ArmToStow(""));
	}
};

#endif /* SRC_COMMANDS_AUTOSCALECOMMANDGROUP_H_ */
