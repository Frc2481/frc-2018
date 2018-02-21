/*
 * ArmIntakeBackPosToggleCommand.h
 *
 *  Created on: Feb 19, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMINTAKEBACKPOSTOGGLECOMMAND_H_
#define SRC_COMMANDS_ARMINTAKEBACKPOSTOGGLECOMMAND_H_

#include "CommandBase.h"
#include "Commands/ArmBaseCommand.h"
#include "Subsystems/Arm.h"
#include "RobotParameters.h"

class ArmIntakeBackPosToggleCommand : public InstantCommand {
private:
	Command* m_armIntake1Back;
	Command* m_armIntake2Back;
	Command* m_armIntake3Back;

public:
	ArmIntakeBackPosToggleCommand() : InstantCommand("ArmIntakeBackPosToggleCommand") {
		m_armIntake1Back = new ArmToIntakeBack("");
		m_armIntake2Back = new ArmToIntake2Back("");
		m_armIntake3Back = new ArmToIntake3Back("");
	}
	virtual ~ArmIntakeBackPosToggleCommand() {}
	void Initialize() {
		double pivotAngle = CommandBase::m_arm->GetDesiredPivotAngle().getDegrees();
		if(fabs(pivotAngle - ArmToIntakeBack::k_pivotAngle) < .0001) {
			m_armIntake2Back->Start();
		}
		else if(fabs(pivotAngle - ArmToIntake2Back::k_pivotAngle) < .0001) {
			m_armIntake3Back->Start();
		}
		else {
			m_armIntake1Back->Start();
		}
	}
};

#endif /* SRC_COMMANDS_ARMINTAKEBACKPOSTOGGLECOMMAND_H_ */
