/*
 * ArmRetractWhenExtendedCommand.h
 *
 *  Created on: Feb 10, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMRETRACTWHENEXTENDEDCOMMAND_H_
#define SRC_COMMANDS_ARMRETRACTWHENEXTENDEDCOMMAND_H_

#include "CommandBase.h"

class ArmRetractWhenExtendedCommand : public CommandBase {
public:
	ArmRetractWhenExtendedCommand() : CommandBase("ArmRetractWhenExtendedCommand") {

	}
	virtual ~ArmRetractWhenExtendedCommand(){}

	void Initialize() {
		if(m_arm->GetExtensionPosition() > 10) {
			m_arm->SetDesiredExtension(0);
		}
	}

	bool IsFinished() {
		return m_arm->GetExtensionPosition() < 10;
	}
};

#endif /* SRC_COMMANDS_ARMRETRACTWHENEXTENDEDCOMMAND_H_ */
