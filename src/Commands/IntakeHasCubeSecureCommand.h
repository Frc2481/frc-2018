/*
 * IntakeHasCubeSecureCommand.h
 *
 *  Created on: Apr 17, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_INTAKEHASCUBESECURECOMMAND_H_
#define SRC_COMMANDS_INTAKEHASCUBESECURECOMMAND_H_

#include "CommandBase.h"

class IntakeHasCubeSecureCommand : public CommandBase{
public:
	IntakeHasCubeSecureCommand() : CommandBase("IntakeHasCubeSecureCommand") {}
	virtual ~IntakeHasCubeSecureCommand() {}
	void Initialize() {

	}

	bool IsFinished() {
		return m_intake->HasCubeSecure();
	}
};

#endif /* SRC_COMMANDS_INTAKEHASCUBESECURECOMMAND_H_ */
