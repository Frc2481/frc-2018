/*
 * IntakeGrabCubeCommandGroup.h
 *
 *  Created on: Feb 10, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_INTAKEGRABCUBECOMMANDGROUP_H_
#define SRC_COMMANDS_INTAKEGRABCUBECOMMANDGROUP_H_

#include "CommandBase.h"

class IntakeGrabCubeCommandGroup : public frc::CommandGroup{
public:
	IntakeGrabCubeCommandGroup() : CommandGroup("IntakeGrabCubeCommandGroup") {
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
	}
};

#endif /* SRC_COMMANDS_INTAKEGRABCUBECOMMANDGROUP_H_ */
