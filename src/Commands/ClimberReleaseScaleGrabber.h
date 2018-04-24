/*
 * ClimberReleaseScaleGrabber.h
 *
 *  Created on: Apr 22, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_CLIMBERRELEASESCALEGRABBER_H_
#define SRC_COMMANDS_CLIMBERRELEASESCALEGRABBER_H_

#include "CommandBase.h"

class ClimberReleaseScaleGrabber : public CommandBase{
public:
	ClimberReleaseScaleGrabber() : CommandBase("ClimberReleaseScaleGrabber"){}

	void Initialize() {
		if(m_driveTrain->GetClimberStep() >= 0) {
			m_driveTrain->SetScaleGrabber(0.0);
			if(m_driveTrain->GetClimberStep() <= 1) {
				m_driveTrain->SetClimberStep(1);
			}
		}
	}

	void End() {
		m_driveTrain->SetScaleGrabber(0.5);
	}

	bool IsFinished() {
		return false;
	}

	void Interrupted() {
		End();
	}
};

#endif /* SRC_COMMANDS_CLIMBERRELEASESCALEGRABBER_H_ */
