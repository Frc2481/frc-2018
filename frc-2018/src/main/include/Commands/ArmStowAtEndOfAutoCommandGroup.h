/*
 * ArmStowAtEndOfAutoCommandGroup.h
 *
 *  Created on: Feb 22, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_ARMSTOWATENDOFAUTOCOMMANDGROUP_H_
#define SRC_COMMANDS_ARMSTOWATENDOFAUTOCOMMANDGROUP_H_

#include "frc/Commands/CommandGroup.h"
#include "ArmBaseCommand.h"

//doesn't work

//use at start of all autos

class ArmStowAtEndOfAutoCommandGroup : public frc::CommandGroup{
public:
	ArmStowAtEndOfAutoCommandGroup() : CommandGroup("ArmStowAtEndOfAutoCommandGroup") {
		AddSequential(new frc::WaitCommand(14));
		AddSequential(new ArmToStow(""));
	}
};

#endif /* SRC_COMMANDS_ARMSTOWATENDOFAUTOCOMMANDGROUP_H_ */
