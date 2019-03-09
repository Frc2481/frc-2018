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
	int m_extensionThreshold;
public:
	ArmRetractWhenExtendedCommand(int desiredPivot) : CommandBase("ArmRetractWhenExtendedCommand") {
		m_desiredPivot = desiredPivot;

		m_extensionThreshold = 20;
	}
	virtual ~ArmRetractWhenExtendedCommand(){}

	void Initialize() {
		//front high zone
		m_skip = false;
		m_extensionThreshold = 20;

		double currentAngle = m_arm->GetPivotAngle().getDegrees();

		//skip retract arm if desired & current pivot extension in same zone
		if (currentAngle <  39 &&
			currentAngle >  0 &&
			m_desiredPivot < 39 &&
			m_desiredPivot > 0) {
				m_skip = true;
		}
		else if (currentAngle >  -39 &&
				 currentAngle <  0 &&
				 m_desiredPivot > -39 &&
				 m_desiredPivot < 0) {
				m_skip = true;
		}
		else {
			m_arm->SetDesiredExtension(0);
		}

		//for long arm movements, don't require arm full retract when pivot
		if(currentAngle > -90 && currentAngle < 0 && m_desiredPivot > 90 && m_desiredPivot < 180) {
			m_extensionThreshold = 30;
		}
		else if(currentAngle < 90 && currentAngle > 0 && m_desiredPivot < -90 && m_desiredPivot > -180) {
			m_extensionThreshold = 30;
		}
	}

	void Execute() {
		m_arm->SetExtensionPosition(m_arm->GetAllowedExtensionPos());
	}

	bool IsFinished() {
		return (m_arm->GetExtensionPosition() < m_extensionThreshold) || (m_skip == true);
	}
};

#endif /* SRC_COMMANDS_ARMRETRACTWHENEXTENDEDCOMMAND_H_ */
