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

class AutoScale1CommandGroup : public CommandGroup{
public:
	AutoScale1CommandGroup(std::string path) : CommandGroup("AutoScale1CommandGroup"){
		AddSequential(new DriveTrainFollowPath(path)); //to scale
		AddSequential(new ArmToMidScaleFront("")); //front or back?
		AddSequential(new IntakeReleaseCubeCommandGroup(1.0));
	}
};

#endif /* SRC_COMMANDS_AUTOSCALE1COMMANDGROUP_H_ */
