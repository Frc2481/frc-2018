/*
 * AutoScale1CommandGroup.h
 *
 *  Created on: Feb 8, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOSCALECOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOSCALECOMMANDGROUP_H_

#include "Commands/DriveTrainFollowPath.h"
#include "CommandBase.h"
#include "Commands/IntakeReleaseCubeCommandGroup.h"
#include "Commands/ObserverResetPosCommand.h"
#include "Commands/ArmExtentionMotionScaling.h"
#include "Commands/ArmBaseCommand.h"

template<class ARM>
class AutoScaleCommandGroup : public frc::CommandGroup{
public:
	AutoScaleCommandGroup(std::string path) : CommandGroup("AutoScaleCommandGroup"){
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ARM(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddSequential(new DriveTrainFollowPath(path));
		AddSequential(new PrintCommand("FollwPath"));

		AddParallel(new IntakeReleaseCubeCommandGroup(0.5), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new PrintCommand("ArmExtentionMotionScaling"));
//		AddSequential(new ArmToStow(""));
	}
};

#endif /* SRC_COMMANDS_AUTOSCALECOMMANDGROUP_H_ */
