/*
 * ArmRetractCommand.h
 *
 *  Created on: Jan 25, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMRETRACTCOMMAND_H_
#define SRC_COMMANDS_ARMRETRACTCOMMAND_H_

#include "CommandBase.h"

class ArmRetractCommand : public CommandBase{
public:
	ArmRetractCommand() : CommandBase("ArmRetractCommand"){}
	virtual ~ArmRetractCommand(){}

	void Initialize() {
		m_arm->SetExtensionOpenLoop(-1);
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

#endif /* SRC_COMMANDS_ARMRETRACTCOMMAND_H_ */
