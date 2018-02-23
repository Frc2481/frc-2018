/*
 * AutoScale1CommandGroup.h
 *
 *  Created on: Feb 8, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOSCALE1COMMANDGROUP_H_
#define SRC_COMMANDS_AUTOSCALE1COMMANDGROUP_H_

#include "CommandBase.h"
#include "Commands/IntakeReleaseCubeCommandGroup.h"
#include "Commands/ObserverResetPosCommand.h"
#include "Commands/ArmExtentionMotionScaling.h"

class AutoScale1CommandGroup : public CommandGroup{
public:
	AutoScale1CommandGroup(std::string path) : CommandGroup("AutoScale1CommandGroup"){
//		AddSequential(new ArmExtentionMotionScaling(0.3));
//		AddParallel(new ArmToHighScaleFront("")); //front
		AddSequential(new DriveTrainFollowPath(path)); //to scale
		AddSequential(new IntakeReleaseCubeCommandGroup(0.7));
//		AddSequential(new ArmExtentionMotionScaling(1.0));
	}
};

#endif /* SRC_COMMANDS_AUTOSCALE1COMMANDGROUP_H_ */
