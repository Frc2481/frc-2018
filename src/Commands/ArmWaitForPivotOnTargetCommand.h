/*
 * ArmWaitForPivotOnTargetCommand.h
 *
 *  Created on: Feb 24, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMWAITFORPIVOTONTARGETCOMMAND_H_
#define SRC_COMMANDS_ARMWAITFORPIVOTONTARGETCOMMAND_H_

#include "CommandBase.h"

class ArmWaitForPivotOnTargetCommand : public CommandBase{
public:
	ArmWaitForPivotOnTargetCommand() : CommandBase("ArmWaitForPivotOnTargetCommand"){}
	virtual ~ArmWaitForPivotOnTargetCommand(){}
	bool IsFinished() {
		return fabs(m_arm->GetDesiredPivotAngle().getDegrees() - m_arm->GetPivotAngle().getDegrees()) < 1;
	}
};

#endif /* SRC_COMMANDS_ARMWAITFORPIVOTONTARGETCOMMAND_H_ */
