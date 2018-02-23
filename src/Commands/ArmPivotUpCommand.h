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

class ArmPivotUpCommand : public CommandBase{
public:
	ArmPivotUpCommand() : CommandBase("ArmPivotUpCommand"){}
	virtual ~ArmPivotUpCommand(){}

	void Initialize() {
		if(CommandBase::m_arm->GetPivotAngle().getDegrees() > 0) {
			CommandBase::m_arm->SetPivotAngle(Rotation2D::fromDegrees(CommandBase::m_arm->GetPivotAngle().getDegrees() + 2));
		}
		else {
			CommandBase::m_arm->SetPivotAngle(Rotation2D::fromDegrees(CommandBase::m_arm->GetPivotAngle().getDegrees() - 2));
		}
	}

	void Execute() {
//		ArmBaseCommand();
	}

	bool IsFinished() {
		return false;
	}

};

#endif /* SRC_COMMANDS_ARMPIVOTUPCOMMAND_H_ */
