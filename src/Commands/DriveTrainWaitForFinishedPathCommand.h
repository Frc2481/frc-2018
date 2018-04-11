/*
 * DriveTrainWaitForFinishedPathCommand.h
 *
 *  Created on: Feb 26, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_DRIVETRAINWAITFORFINISHEDPATHCOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINWAITFORFINISHEDPATHCOMMAND_H_

#include "CommandBase.h"

class DriveTrainWaitForFinishedPathCommand : public CommandBase{
public:
	DriveTrainWaitForFinishedPathCommand() : CommandBase("DriveTrainWaitForFinishedPathCommand") {

	}
	virtual ~DriveTrainWaitForFinishedPathCommand(){}

	bool IsFinished() {
		return m_pathFollower->IsFinished();
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINWAITFORFINISHEDPATHCOMMAND_H_ */
