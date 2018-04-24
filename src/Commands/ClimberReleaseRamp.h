/*
 * ClimberReleaseRamp.h
 *
 *  Created on: Apr 23, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_CLIMBERRELEASERAMP_H_
#define SRC_COMMANDS_CLIMBERRELEASERAMP_H_

#include "CommandBase.h"

class ClimberReleaseRamp : public CommandBase{
public:
	ClimberReleaseRamp() : CommandBase("ClimberReleaseRamp") {
		Requires(m_driveTrain.get());
	}
	virtual ~ClimberReleaseRamp() {}

	void Initialize() {
		if(m_driveTrain->GetClimberStep() >= 1) {
			m_driveTrain->EngagePTO();
			m_driveTrain->SetFarWinchSpeed(-1); //change speed later
			if(m_driveTrain->GetClimberStep() <= 2) {
				m_driveTrain->SetClimberStep(2);
			}
		}
	}
	void End() {
		m_driveTrain->SetFarWinchSpeed(0);
	}
	void Interrupted() {
		End();
	}
	bool IsFinished() {
		return false;
	}
};

#endif /* SRC_COMMANDS_CLIMBERRELEASERAMP_H_ */
