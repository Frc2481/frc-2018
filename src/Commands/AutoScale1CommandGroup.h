/*
 * AutoScale1CommandGroup.h
 *
 *  Created on: Feb 8, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOSCALE1COMMANDGROUP_H_
#define SRC_COMMANDS_AUTOSCALE1COMMANDGROUP_H_

#include "CommandBase.h"

class AutoScale1CommandGroup : public CommandGroup{
public:
	AutoScale1CommandGroup(const std::string name, path) : CommandGroup(name){
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new ArmToStow());
		//follow path
		//potential wait command
		AddParallel(new ArmToMidScaleFront());
		AddSequential(new IntakeRollerUnloadCommand());
		AddSequential(new WaitCommand(.3)); //change time or use something different
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new ArmToStow());
		AddSequential(new AutoGetCubeCommandGroup());
	}
};

#endif /* SRC_COMMANDS_AUTOSCALE1COMMANDGROUP_H_ */
