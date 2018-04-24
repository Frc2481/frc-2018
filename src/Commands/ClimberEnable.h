/*
 * ClimberEnable.h
 *
 *  Created on: Apr 23, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_CLIMBERENABLE_H_
#define SRC_COMMANDS_CLIMBERENABLE_H_

#include "CommandBase.h"

class ClimberEnable : public CommandBase{
public:
	ClimberEnable() : CommandBase("ClimberEnable") {}
	virtual ~ClimberEnable() {}

	void Initialize() {
		m_driveTrain->SetClimberStep(0);
	}

	bool IsFinished() {
		return false;
	}
};

#endif /* SRC_COMMANDS_CLIMBERENABLE_H_ */
