/*
 * DriveTrainFarWinchCommand.h
 *
 *  Created on: Feb 10, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_DRIVETRAINFARWINCHCOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINFARWINCHCOMMAND_H_

#include "CommandBase.h"

class DriveTrainFarWinchCommand : public CommandBase{
public:
	DriveTrainFarWinchCommand() : CommandBase("DriveTrainFarWinchCommand") {
		Requires(m_driveTrain.get());
	}
	virtual ~DriveTrainFarWinchCommand(){}

	void Initialize() {
		m_driveTrain->SetFarWinchSpeed(-1);
	}
	void End() {
		m_driveTrain->SetFarWinchSpeed(0);
	}
	void Interrupted() {
		End();
	}
	bool IsFinished() {
		return false;
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINFARWINCHCOMMAND_H_ */
