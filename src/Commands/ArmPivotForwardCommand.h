/*
 * ArmPivotCommand.h
 *
 *  Created on: Jan 29, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMPIVOTFORWARDCOMMAND_H_
#define SRC_COMMANDS_ARMPIVOTFORWARDCOMMAND_H_

#include "CommandBase.h"
#include "Subsystems/Arm.h"

class ArmPivotForwardCommand : public CommandBase{
public:
	ArmPivotForwardCommand() : CommandBase("ArmPivotForwardCommand"){}
	virtual ~ArmPivotForwardCommand(){}

	void Initialize() {
		m_arm->SetPivotOpenLoop(.25);
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

#endif /* SRC_COMMANDS_ARMPIVOTFORWARDCOMMAND_H_ */
