/*
 * ArmPivotUpCommand.h
 *
 *  Created on: Jan 29, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMPIVOTUPCOMMAND_H_
#define SRC_COMMANDS_ARMPIVOTUPCOMMAND_H_

#include "CommandBase.h"
#include "Subsystems/Arm.h"
#include "Commands/ArmBaseCommand.h"
#include "../XboxController.h"
#include "Components/Joystick2481.h"

class ArmPivotUpCommand : public CommandBase{
public:
	ArmPivotUpCommand() : CommandBase("ArmPivotUpCommand"){
		Requires(m_arm.get());
	}
	virtual ~ArmPivotUpCommand(){}

	void Initialize() {}

	void Execute() {
		if(!m_arm->IsExtensionZeroed()) {
			m_arm->SetExtensionOpenLoop(oi->GetOperatorStick()->GetRawAxis(XB_RIGHT_Y_AXIS) * -0.25);
		}
		else if(!m_arm->IsPivotZeroed()) {
			m_arm->SetPivotOpenLoop(oi->GetOperatorStick()->GetRawAxis(XB_RIGHT_Y_AXIS) * -0.25);
		}
		else{
			double increment = oi->GetOperatorStick()->GetRawAxis(XB_RIGHT_Y_AXIS) * -0.5;
			double currentPivotAngle = m_arm->GetDesiredPivotAngle().getDegrees();
			double currentExtentionPos = m_arm->GetDesiredExtension();

			if(currentPivotAngle > 0) {
				m_arm->SetPivotAngle(Rotation2D::fromDegrees(std::max(currentPivotAngle - increment, (double)ArmToHighScale2Front::k_pivotAngle)));
				m_arm->SetDesiredExtension(currentExtentionPos + increment);
			}
			else {
				m_arm->SetPivotAngle(Rotation2D::fromDegrees(std::min(currentPivotAngle + increment, (double)ArmToHighScale2Back::k_pivotAngle)));
				m_arm->SetDesiredExtension(currentExtentionPos + increment);
			}
		}
	}

	bool IsFinished() {
		return false;
	}

	void Interrupted() {
		if (m_arm->IsPivotZeroed() == false) {
			m_arm->SetPivotOpenLoop(0);
		}
		else if(m_arm->IsExtensionZeroed() == false) {
			m_arm->SetExtensionOpenLoop(0);
		}
	}

};

#endif /* SRC_COMMANDS_ARMPIVOTUPCOMMAND_H_ */
