/*
 * ClimberStart.h
 *
 *  Created on: Apr 23, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_CLIMBERSTART_H_
#define SRC_COMMANDS_CLIMBERSTART_H_

#include "Commands/CommandGroup.h"
#include "Commands/ArmBaseCommand.h"
#include "Commands/ClimberEnable.h"

class ClimberStart : public CommandGroup{
public:
	ClimberStart() : CommandGroup("ClimberStart") {
		AddSequential(new ArmToClimbPos(""));
		AddSequential(new ClimberEnable());
	}

};

#endif /* SRC_COMMANDS_CLIMBERSTART_H_ */
