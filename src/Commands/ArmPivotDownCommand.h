/*
 * ArmPivotDownCommand.h
 *
 *  Created on: Jan 29, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMPIVOTDOWNCOMMAND_H_
#define SRC_COMMANDS_ARMPIVOTDOWNCOMMAND_H_

#include "CommandBase.h"
#include "Subsystems/Arm.h"
#include "Commands/ArmBaseCommand.h"
#include "../XboxController.h"
#include "Components/Joystick2481.h"

class ArmPivotDownCommand : public CommandBase{
public:
	ArmPivotDownCommand() : CommandBase("ArmPivotDownCommand"){
		Requires(m_arm.get());
	}
	virtual ~ArmPivotDownCommand(){}

	void Initialize() {}

	void Execute() {
		double increment = oi->GetOperatorStick()->GetRawAxis(XB_RIGHT_Y_AXIS) * 0.5;
		double currentPivotAngle = m_arm->GetDesiredPivotAngle().getDegrees();

		if(currentPivotAngle > 0) {
			m_arm->SetPivotAngle(Rotation2D::fromDegrees(std::min(currentPivotAngle + increment, (double)ArmToIntakeFront::k_pivotAngle)));
		}
		else {
			m_arm->SetPivotAngle(Rotation2D::fromDegrees(std::max(currentPivotAngle - increment, (double)ArmToIntakeBack::k_pivotAngle)));
		}
	}

	bool IsFinished() {
		return false;
	}
};

#endif /* SRC_COMMANDS_ARMPIVOTDOWNCOMMAND_H_ */
