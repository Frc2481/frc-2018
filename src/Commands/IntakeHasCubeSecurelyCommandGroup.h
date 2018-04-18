/*
 * IntakeHasCubeSecurelyCommandGroup.h
 *
 *  Created on: Apr 17, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_INTAKEHASCUBESECURELYCOMMANDGROUP_H_
#define SRC_COMMANDS_INTAKEHASCUBESECURELYCOMMANDGROUP_H_

#include "Commands/CommandGroup.h"
#include "Commands/IntakeHasCubeSecureCommand.h"
#include "Commands/IntakeStopCubeCommandGroup.h"

class IntakeHasCubeSecurelyCommandGroup : public CommandGroup {
public:
	IntakeHasCubeSecurelyCommandGroup() : CommandGroup("IntakeHasCubeSecurelyCommandGroup") {
		AddSequential(new IntakeHasCubeSecureCommand());
		AddParallel(new IntakeClampCloseCommand());
		AddParallel(new IntakeStopCubeCommandGroup());
	}
};

#endif /* SRC_COMMANDS_INTAKEHASCUBESECURELYCOMMANDGROUP_H_ */
