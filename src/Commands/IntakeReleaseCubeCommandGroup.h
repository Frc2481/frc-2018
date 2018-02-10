/*
 * IntakeReleaseCubeCommandGroup.h
 *
 *  Created on: Feb 8, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_INTAKERELEASECUBECOMMANDGROUP_H_
#define SRC_COMMANDS_INTAKERELEASECUBECOMMANDGROUP_H_

#include "CommandBase.h"

class IntakeReleaseCubeCommandGroup : public CommandGroup {
public:
	IntakeReleaseCubeCommandGroup() : CommandGroup("IntakeReleaseCubeCommandGroup") {
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerUnloadCommand());
		AddSequential(new WaitCommand(1));
		AddSequential(new IntakeRollerOffCommand());
	}
};

#endif /* SRC_COMMANDS_INTAKERELEASECUBECOMMANDGROUP_H_ */
