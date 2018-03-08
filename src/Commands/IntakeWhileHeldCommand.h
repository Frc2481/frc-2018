/*
 * IntakeWhileHeldCommandGroup.h
 *
 *  Created on: Mar 1, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_INTAKEWHILEHELDCOMMAND_H_
#define SRC_COMMANDS_INTAKEWHILEHELDCOMMAND_H_

#include "CommandBase.h"

class IntakeWhileHeldCommand : public CommandBase{
public:
	IntakeWhileHeldCommand() : CommandBase("IntakeWhileHeldCommand"){
		Requires(CommandBase::m_intake.get());
	}
	virtual ~IntakeWhileHeldCommand(){}

	void Initialize() {
		m_intake->OpenClamp();
		m_intake->RollerLoad(1);
	}

	bool IsFinished() {
		return false;
	}

	void Interrupted() {
		m_intake->RollerOff();
		m_intake->CloseClamp();
	}



};

#endif /* SRC_COMMANDS_INTAKEWHILEHELDCOMMAND_H_ */
