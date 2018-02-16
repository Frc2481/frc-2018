/*
 * ArmPivotReverseCommand.h
 *
 *  Created on: Jan 29, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMPIVOTREVERSECOMMAND_H_
#define SRC_COMMANDS_ARMPIVOTREVERSECOMMAND_H_

#include "CommandBase.h"
#include "Subsystems/Arm.h"

class ArmPivotReverseCommand : public CommandBase{
public:
	ArmPivotReverseCommand() : CommandBase("ArmPivotReverseCommand"){}
	virtual ~ArmPivotReverseCommand(){}

	void Initialize() {
		m_arm->SetPivotOpenLoop(-.25);
	}
	bool IsFinished() {
		return false;
	}

	void Interrupted() {
		End();
	}
	void End() {
		m_arm->SetPivotOpenLoop(0.0);
	}
};

#endif /* SRC_COMMANDS_ARMPIVOTREVERSECOMMAND_H_ */
