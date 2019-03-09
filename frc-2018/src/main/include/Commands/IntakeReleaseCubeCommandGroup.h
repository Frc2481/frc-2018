/*
 * IntakeReleaseCubeCommandGroup.h
 *
 *  Created on: Feb 8, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_INTAKERELEASECUBECOMMANDGROUP_H_
#define SRC_COMMANDS_INTAKERELEASECUBECOMMANDGROUP_H_

#include "CommandBase.h"
#include "Commands/IntakeClampOpenCommand.h"
#include "Commands/IntakeRollerUnloadCommand.h"
#include "Commands/IntakeRollerOffCommand.h"


class IntakeReleaseCubeCommandGroup : public CommandGroup {
public:
	IntakeReleaseCubeCommandGroup(double speed) : CommandGroup("IntakeReleaseCubeCommandGroup") {
		Requires(CommandBase::m_intake.get());
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerUnloadCommand(speed));
		AddSequential(new WaitCommand(1));
		AddSequential(new IntakeRollerOffCommand());
	}
};

#endif /* SRC_COMMANDS_INTAKERELEASECUBECOMMANDGROUP_H_ */
