/*
 * ArmPivotEncoderZeroCommand.h
 *
 *  Created on: Jan 29, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMPIVOTENCODERZEROCOMMAND_H_
#define SRC_COMMANDS_ARMPIVOTENCODERZEROCOMMAND_H_

#include "CommandBase.h"
#include "Subsystems/Arm.h"

class ArmPivotEncoderZeroCommand : public frc::InstantCommand{
public:
	ArmPivotEncoderZeroCommand() : InstantCommand("ArmPivotEncoderZeroCommand") {
		SetRunWhenDisabled(true);
	}
	virtual ~ArmPivotEncoderZeroCommand(){}
	void Initialize() {
		CommandBase::m_arm->ZeroPivot();
	}
};

#endif /* SRC_COMMANDS_ARMPIVOTENCODERZEROCOMMAND_H_ */
