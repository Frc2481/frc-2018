/*
 * ClimberReverseSpringHooks.h
 *
 *  Created on: Apr 22, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_CLIMBERREVERSESPRINGHOOKS_H_
#define SRC_COMMANDS_CLIMBERREVERSESPRINGHOOKS_H_

#include "CommandBase.h"

class ClimberReverseSpringHooks : public CommandBase{
public:
	ClimberReverseSpringHooks() : CommandBase("ClimberReverseSpringHooks") {}
//	virtual ~ClimberReverseSpringHooks();

	void Initialize() {
		m_driveTrain->SetSpringHooks(1.0);
	}

	void End() {
		m_driveTrain->SetSpringHooks(0.5);
	}

	bool IsFinished() {
		return false;
	}

	void Interrupted() {
		End();
	}
};

#endif /* SRC_COMMANDS_CLIMBERREVERSESPRINGHOOKS_H_ */
