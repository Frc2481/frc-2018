/*
 * ClimberReverseScaleGrabber.h
 *
 *  Created on: Apr 22, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_CLIMBERREVERSESCALEGRABBER_H_
#define SRC_COMMANDS_CLIMBERREVERSESCALEGRABBER_H_

#include "CommandBase.h"

class ClimberReverseScaleGrabber : public CommandBase{
public:
	ClimberReverseScaleGrabber() : CommandBase("ClimberReverseScaleGrabber") {}
	virtual ~ClimberReverseScaleGrabber() {}

	void Initialize() {
		m_driveTrain->SetScaleGrabber(1.0);
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

#endif /* SRC_COMMANDS_CLIMBERREVERSESCALEGRABBER_H_ */
