/*
 * DriveTrainNearWinchCommand.h
 *
 *  Created on: Feb 10, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_DRIVETRAINNEARWINCHCOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINNEARWINCHCOMMAND_H_

#include "CommandBase.h"

class DriveTrainNearWinchCommand : public CommandBase{
public:
	DriveTrainNearWinchCommand() : CommandBase("DriveTrainNearWinchCommand") {
		Requires(m_driveTrain.get());
	}
	virtual ~DriveTrainNearWinchCommand(){}

	void Initialize() {
		m_driveTrain->SetNearWinchSpeed(0.8);
	}
	void End() {
		m_driveTrain->SetNearWinchSpeed(0);
	}
	void Interrupted() {
		End();
	}
	bool IsFinished() {
		return false;
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINNEARWINCHCOMMAND_H_ */
