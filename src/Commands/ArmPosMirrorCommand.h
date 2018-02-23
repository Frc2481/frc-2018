/*
 * ArmPosMirrorCommand.h
 *
 *  Created on: Feb 19, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMPOSMIRRORCOMMAND_H_
#define SRC_COMMANDS_ARMPOSMIRRORCOMMAND_H_

#include "CommandBase.h"
#include "Subsystems/Arm.h"

class ArmPosMirrorCommand : public InstantCommand{
public:
	ArmPosMirrorCommand() : InstantCommand("ArmPosMirrorCommand"){

	}
	virtual ~ArmPosMirrorCommand(){}

	void Initialize() {
//		if(CommandBase::m_arm->GetDesiredPivotAngle() //  ) {
//
//		}
	}
};

#endif /* SRC_COMMANDS_ARMPOSMIRRORCOMMAND_H_ */
