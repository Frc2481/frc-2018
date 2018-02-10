/*
 * ArmExtensionEncoderZeroCommand.h
 *
 *  Created on: Jan 25, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMEXTENSIONENCODERZEROCOMMAND_H_
#define SRC_COMMANDS_ARMEXTENSIONENCODERZEROCOMMAND_H_

#include "CommandBase.h"
#include "Subsystems/Arm.h"

class ArmExtensionEncoderZeroCommand : public InstantCommand{
public:
	ArmExtensionEncoderZeroCommand() : InstantCommand("ArmExtensionEncoderZeroCommand") {
		SetRunWhenDisabled(true);
	}

	void Initialize() {
		CommandBase::m_arm->ZeroExtension();
	}
};

#endif /* SRC_COMMANDS_ARMEXTENSIONENCODERZEROCOMMAND_H_ */
