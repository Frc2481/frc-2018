/*
 * AutoSwitch1CommandGroup.h
 *
 *  Created on: Feb 8, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOSWITCH1COMMANDGROUP_H_
#define SRC_COMMANDS_AUTOSWITCH1COMMANDGROUP_H_

#include "CommandBase.h"

class AutoSwitch1CommandGroup : public CommandGroup{
public:
	AutoSwitch1CommandGroup(const std::string name, path) : CommandBase(name){
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new ArmToStow());
		//path to switch
		//potential wait command
		AddParallel(new ArmToSwitchFront());
		AddSequential(new IntakeRollerUnloadCommand());
		AddSequential(new WaitCommand(.3)); //change time or use something different
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new ArmToStow());
	}
};

#endif /* SRC_COMMANDS_AUTOSWITCH1COMMANDGROUP_H_ */
