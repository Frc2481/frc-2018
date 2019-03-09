/*
 * AutoCommand.h
 *
 *  Created on: Feb 6, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOCOMMAND_H_
#define SRC_COMMANDS_AUTOCOMMAND_H_

#include "CommandBase.h"

class AutoCommand : public CommandBase{
public:
	AutoCommand() : CommandBase("AutoCommand"){}
	virtual ~AutoCommand(){}
	bool IsFinished() {
		return true;
	}
};

#endif /* SRC_COMMANDS_AUTOCOMMAND_H_ */
