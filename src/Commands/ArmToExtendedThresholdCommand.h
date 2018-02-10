/*
 * ArmToExtendedThresholdCommand.h
 *
 *  Created on: Jan 26, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMTOEXTENDEDTHRESHOLDCOMMAND_H_
#define SRC_COMMANDS_ARMTOEXTENDEDTHRESHOLDCOMMAND_H_

#include "CommandBase.h"
#include "Subsystems/Arm.h"

class ArmToExtendedThresholdCommand : public CommandBase{
public:
	ArmToExtendedThresholdCommand() : CommandBase("ArmToExtendedThresholdCommand") {}
	virtual ~ArmToExtendedThresholdCommand() {}
	void Initialize() {
		CommandBase::m_arm->SetExtensionPostion(m_arm->ConvertEncTicksToInches(15777)); // far threshold - 2000
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

#endif /* SRC_COMMANDS_ARMTOEXTENDEDTHRESHOLDCOMMAND_H_ */
