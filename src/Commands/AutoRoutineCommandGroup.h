/*
 * AutoRoutineCommandGroup.h
 *
 *  Created on: Feb 8, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOROUTINEGROUPCOMMAND_H_
#define SRC_COMMANDS_AUTOROUTINEGROUPCOMMAND_H_

#include "CommandBase.h"

class AutoRoutineCommandGroup : public CommandGroup{
public:
	AutoRoutineCommandGroup() : CommandGroup("AutoRoutineCommandGroup"){

	}
};

#endif /* SRC_COMMANDS_AUTOROUTINEGROUPCOMMAND_H_ */
