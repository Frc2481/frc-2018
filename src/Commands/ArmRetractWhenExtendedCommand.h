/*
 * ArmRetractWhenExtendedCommand.h
 *
 *  Created on: Feb 10, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMRETRACTWHENEXTENDEDCOMMAND_H_
#define SRC_COMMANDS_ARMRETRACTWHENEXTENDEDCOMMAND_H_

#include "CommandBase.h"
#include "Subsystems/Arm.h"

class ArmRetractWhenExtendedCommand : public CommandBase {
private:
	bool m_skip;
	int m_desiredPivot;
public:
	ArmRetractWhenExtendedCommand(int desiredPivot) : CommandBase("ArmRetractWhenExtendedCommand") {
		m_desiredPivot = desiredPivot;
		m_skip = false;
	}
	virtual ~ArmRetractWhenExtendedCommand(){}

	void Initialize() {
		//front high zone
		//skip retract arm if desired & current pivot extension in same zone
		if (m_arm->GetPivotAngle().getDegrees() <  39 &&
			m_arm->GetPivotAngle().getDegrees() >  0 &&
			m_desiredPivot < 39 &&
			m_desiredPivot > 0) {
				m_skip = true;
		}
		else if (m_arm->GetPivotAngle().getDegrees() >  -39 &&
				 m_arm->GetPivotAngle().getDegrees() <  0 &&
				 m_desiredPivot > -39 &&
				 m_desiredPivot < 0) {
				m_skip = true;
		}
		else {
			m_arm->SetDesiredExtension(0);
		}
	}

	bool IsFinished() {
		return (m_arm->GetExtensionPosition() < 10) || (m_skip == true);
	}
};

#endif /* SRC_COMMANDS_ARMRETRACTWHENEXTENDEDCOMMAND_H_ */
