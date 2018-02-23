/*
 * DriveTrainWaitForFieldXCommand.h
 *
 *  Created on: Feb 22, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINWAITFORFIELDXCOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINWAITFORFIELDXCOMMAND_H_

#include "CommandBase.h"

class DriveTrainWaitForFieldXCommand : public CommandBase{
private:
	double m_xPos;
	bool m_leftToRight;

public:
	DriveTrainWaitForFieldXCommand(double xPos) : CommandBase("DriveTrainWaitForFieldXCommand") {
		m_xPos = xPos;
		m_leftToRight = true;
	}
	virtual ~DriveTrainWaitForFieldXCommand(){}

	void Initialize() {
		RigidTransform2D pose = m_driveTrain->GetObserver()->GetRobotPos(RobotController::GetFPGATime());
		m_leftToRight = pose.getTranslation().getX() < m_xPos;
	}

	bool IsFinished() {
		RigidTransform2D pose = m_driveTrain->GetObserver()->GetRobotPos(RobotController::GetFPGATime());
		if(m_leftToRight) {
			return pose.getTranslation().getX() > m_xPos;
		}
		else {
			return pose.getTranslation().getX() < m_xPos;
		}
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINWAITFORFIELDXCOMMAND_H_ */
