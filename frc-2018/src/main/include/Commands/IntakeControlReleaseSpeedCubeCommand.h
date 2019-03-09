/*
 * IntakeControlReleaseSpeedCubeCommand.h
 *
 *  Created on: Mar 1, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_INTAKECONTROLRELEASESPEEDCUBECOMMAND_H_
#define SRC_COMMANDS_INTAKECONTROLRELEASESPEEDCUBECOMMAND_H_

#include "CommandBase.h"
#include "XboxController2481.h"

class IntakeControlReleaseSpeedCubeCommand : public CommandBase{
public:
	IntakeControlReleaseSpeedCubeCommand() : CommandBase("IntakeControlReleaseSpeedCubeCommand") {
		Requires(m_intake.get());
	}
	virtual ~IntakeControlReleaseSpeedCubeCommand(){}

	void Initialize() {
		m_intake->OpenClamp();
	}

	void Execute() {
		double triggerPos = oi->GetOperatorStick()->GetRawAxis(XB_LEFT_TRIGGER); // Left = Abigail, Right = Abigail

		if (fabs(m_arm->GetDesiredPivotAngle().getDegrees() - ArmToSwitchFront::k_pivotAngle) < 0.1) {
			m_intake->RollerUnload(.6);
		}

		else if (fabs(m_arm->GetDesiredPivotAngle().getDegrees() - ArmToSwitchBack::k_pivotAngle) < 0.1) {
			m_intake->RollerUnload(.6);
		}

		else if(triggerPos > 0) {
			m_intake->RollerUnload(triggerPos + .5);
		}
	}

	bool IsFinished() {
		return false;
	}

	void Interrupted() {
		m_intake->RollerOff();
		m_intake->CloseClamp();
	}
};

#endif /* SRC_COMMANDS_INTAKECONTROLRELEASESPEEDCUBECOMMAND_H_ */
