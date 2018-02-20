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

class ArmPivotDownCommand : public InstantCommand{
public:
	ArmPivotDownCommand() : InstantCommand("ArmPivotDownCommand"){}
	virtual ~ArmPivotDownCommand(){}

	void Initialize() {
		if((CommandBase::m_arm->GetPivotAngle().getDegrees() > 0) &&
				(CommandBase::m_arm->GetPivotAngle().getDegrees() >= 120)) {
			CommandBase::m_arm->SetPivotAngle(Rotation2D::fromDegrees(CommandBase::m_arm->GetPivotAngle().getDegrees() - 2));
		}
		else if((CommandBase::m_arm->GetPivotAngle().getDegrees() <= 0) &&
				(CommandBase::m_arm->GetPivotAngle().getDegrees() <= -118)){
			CommandBase::m_arm->SetPivotAngle(Rotation2D::fromDegrees(CommandBase::m_arm->GetPivotAngle().getDegrees() + 2));
		}
	}
};

#endif /* SRC_COMMANDS_ARMPIVOTDOWNCOMMAND_H_ */
