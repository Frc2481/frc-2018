/*
 * DriveTrainOpenLoopCommand.h
 *
 *  Created on: Feb 10, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_DRIVETRAINOPENLOOPCOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINOPENLOOPCOMMAND_H_

#include "CommandBase.h"

class DriveTrainOpenLoopCommand : public CommandBase{
public:
	DriveTrainOpenLoopCommand() : CommandBase("DriveTrainOpenLoopCommand") {
		Requires(m_driveTrain.get());
	}
	virtual ~DriveTrainOpenLoopCommand(){}
	void Initialize() {
		m_driveTrain->SetOpenLoopSteer(.5);
	}
	void End() {
		m_driveTrain->SetOpenLoopSteer(0);
	}
	bool IsFinished() {
		return false;
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINOPENLOOPCOMMAND_H_ */
