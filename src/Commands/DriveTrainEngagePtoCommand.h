/*
 * DriveTrainEngagePtoCommand.h
 *
 *  Created on: Feb 8, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_DRIVETRAINENGAGEPTOCOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINENGAGEPTOCOMMAND_H_

#include "CommandBase.h"

class DriveTrainEngagePtoCommand : public CommandBase{
public:
	DriveTrainEngagePtoCommand() : CommandBase("DriveTrainEngagePtoCommand"){
	}
	void Initialize() {
		m_driveTrain->EngagePTO();
	}
	bool IsFinished() {
		return false;
	}
	void End() {
		m_driveTrain->DisengagePTO();
	}

	void Interrupted() {
		End();
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINENGAGEPTOCOMMAND_H_ */
