/*
 * ClimberClimb.h
 *
 *  Created on: Apr 23, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_CLIMBERCLIMB_H_
#define SRC_COMMANDS_CLIMBERCLIMB_H_

#include "CommandBase.h"

class ClimberClimb : public CommandBase{
public:
	ClimberClimb() : CommandBase("ClimberClimb") {
		Requires(m_driveTrain.get());
	}
	virtual ~ClimberClimb() {}

	void Initialize() {
		if(m_driveTrain->GetClimberStep() >= 3) {
			m_driveTrain->SetFarWinchSpeed(-1);
			m_driveTrain->SetClimberStep(4);
		}
	}
	void End() {
		m_driveTrain->SetFarWinchSpeed(0.0);
	}
	void Interrupted() {
		End();
	}
	bool IsFinished() {
		return false;
	}
};

#endif /* SRC_COMMANDS_CLIMBERCLIMB_H_ */
