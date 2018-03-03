/*
 * ArmToIntakePos3Command.h
 *
 *  Created on: Feb 27, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMTOINTAKEPOS3COMMAND_H_
#define SRC_COMMANDS_ARMTOINTAKEPOS3COMMAND_H_

#include "CommandBase.h"

class ArmToIntakePos3Command : public InstantCommand{
private:
	Command* m_armIntake3Front;
	Command* m_armIntake3Back;

public:
	ArmToIntakePos3Command() : InstantCommand("ArmToIntakePos3Command") {
		m_armIntake3Front = new ArmToIntake3Front("");
		m_armIntake3Back = new ArmToIntake3Back("");
	}
	virtual ~ArmToIntakePos3Command(){}

	void Initialize() {
		if(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() > 0) {
			m_armIntake3Front->Start();
		}
		else {
			m_armIntake3Back->Start();
		}
	}
};

#endif /* SRC_COMMANDS_ARMTOINTAKEPOS3COMMAND_H_ */
