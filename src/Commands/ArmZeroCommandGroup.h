/*
 * ArmZeroCommandGroup.h
 *
 *  Created on: Feb 17, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMZEROCOMMANDGROUP_H_
#define SRC_COMMANDS_ARMZEROCOMMANDGROUP_H_

#include "Commands/CommandGroup.h"
#include "Commands/ArmExtensionEncoderZeroCommand.h"
#include "Commands/ArmPivotEncoderZeroCommand.h"


class ArmZeroCommandGroup : public CommandGroup{
public:
	ArmZeroCommandGroup() : CommandGroup("ArmZeroCommandGroup"){
		SetRunWhenDisabled(true);
		AddSequential(new ArmExtensionEncoderZeroCommand());
		AddSequential(new ArmPivotEncoderZeroCommand());
	}
};

#endif /* SRC_COMMANDS_ARMZEROCOMMANDGROUP_H_ */
