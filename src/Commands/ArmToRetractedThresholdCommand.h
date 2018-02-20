/*
 * ArmToRetractedThresholdCommand.h
 *
 *  Created on: Jan 26, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMTORETRACTEDTHRESHOLDCOMMAND_H_
#define SRC_COMMANDS_ARMTORETRACTEDTHRESHOLDCOMMAND_H_

#include "CommandBase.h"
#include "Subsystems/Arm.h"

class ArmToRetractedThresholdCommand : public CommandBase {
public:
	ArmToRetractedThresholdCommand() : CommandBase("ArmToRetractedThresholdCommand") {}
	virtual ~ArmToRetractedThresholdCommand() {}
	void Initialize() {
		CommandBase::m_arm->SetExtensionPosition(m_arm->ConvertEncTicksToInches(3000)); // short threshold - 2000
	}
	bool IsFinished() {
		return CommandBase::m_arm->IsExtensionOnTarget();
	}
	void Interrupted() {
		End();
	}
	void End() {

	}

};

#endif /* SRC_COMMANDS_ARMTORETRACTEDTHRESHOLDCOMMAND_H_ */
