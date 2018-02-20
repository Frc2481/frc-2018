/*
 * ArmIntakeFrontPosToggleCommand.h
 *
 *  Created on: Feb 18, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMINTAKEFRONTPOSTOGGLECOMMAND_H_
#define SRC_COMMANDS_ARMINTAKEFRONTPOSTOGGLECOMMAND_H_

#include "CommandBase.h"
#include "Commands/ArmBaseCommand.h"
#include "Subsystems/Arm.h"
#include "RobotParameters.h"

class ArmIntakeFrontPosToggleCommand : public InstantCommand {
private:
	Command* m_armIntake1Front;
	Command* m_armIntake2Front;
	Command* m_armIntake3Front;

public:
	ArmIntakeFrontPosToggleCommand() : InstantCommand("ArmIntakeFrontPosToggleCommand") {
		m_armIntake1Front = new ArmToIntakeFront("");
		m_armIntake2Front = new ArmToIntake2Front("");
		m_armIntake3Front = new ArmToIntake3Front("");
	}
	virtual ~ArmIntakeFrontPosToggleCommand() {}
	void Initialize() {
		if(fabs(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() - RobotParameters::k_intake1FrontAngle) < .0001) {
			m_armIntake2Front->Start();
		}
		else if(fabs(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() - RobotParameters::k_intake2FrontAngle) < .0001) {
			m_armIntake3Front->Start();
		}
		else {
			m_armIntake1Front->Start();
		}
	}
};

#endif /* SRC_COMMANDS_ARMINTAKEFRONTPOSTOGGLECOMMAND_H_ */
