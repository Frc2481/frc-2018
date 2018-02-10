/*
 * IntakeStopCubeCommandGroup.h
 *
 *  Created on: Feb 10, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_INTAKESTOPCUBECOMMANDGROUP_H_
#define SRC_COMMANDS_INTAKESTOPCUBECOMMANDGROUP_H_

#include "CommandBase.h"

class IntakeStopCubeCommandGroup : public CommandGroup{
public:
	IntakeStopCubeCommandGroup() : CommandGroup("IntakeStopCubeCommandGroup") {
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
	}
};

#endif /* SRC_COMMANDS_INTAKESTOPCUBECOMMANDGROUP_H_ */
