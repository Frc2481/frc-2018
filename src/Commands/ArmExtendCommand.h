/*
 * ArmExtendCommand.h
 *
 *  Created on: Jan 25, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMEXTENDCOMMAND_H_
#define SRC_COMMANDS_ARMEXTENDCOMMAND_H_

#include "CommandBase.h"
#include "Subsystems/Arm.h"

class ArmExtendCommand : public CommandBase {
public:
	ArmExtendCommand() : CommandBase("ArmExtendCommand"){

	}
	virtual ~ArmExtendCommand(){}

	void Initialize() {
		m_arm->SetExtensionOpenLoop(1.0);
	}
	bool IsFinished() {
		return false;
	}

	void Interrupted() {
		End();
	}
	void End() {
		m_arm->SetExtensionOpenLoop(0.0);
	}
};

#endif /* SRC_COMMANDS_ARMEXTENDCOMMAND_H_ */
