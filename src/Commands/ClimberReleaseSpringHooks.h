/*
 * ClimberReleaseSpringHooks.h
 *
 *  Created on: Apr 22, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_CLIMBERRELEASESPRINGHOOKS_H_
#define SRC_COMMANDS_CLIMBERRELEASESPRINGHOOKS_H_

#include "CommandBase.h"

class ClimberReleaseSpringHooks : public CommandBase{
public:
	ClimberReleaseSpringHooks() : CommandBase("ClimberReleaseSpringHooks") {}

	void Initialize() {
		if(m_driveTrain->GetClimberStep() >= 2) {
			m_driveTrain->SetSpringHooks(0.0);
			if(m_driveTrain->GetClimberStep() <= 3) {
				m_driveTrain->SetClimberStep(3);
			}
		}
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

#endif /* SRC_COMMANDS_CLIMBERRELEASESPRINGHOOKS_H_ */
