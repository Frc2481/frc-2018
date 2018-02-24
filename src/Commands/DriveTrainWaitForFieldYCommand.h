/*
 * DriveTrainWaitForFieldYCommand.h
 *
 *  Created on: Feb 22, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINWAITFORFIELDYCOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINWAITFORFIELDYCOMMAND_H_

#include "CommandBase.h"

class DriveTrainWaitForFieldYCommand : public CommandBase{
private:
	double m_yPos;
	bool m_upToDown;
public:
	DriveTrainWaitForFieldYCommand(double yPos) : CommandBase("DriveTrainWaitForFieldYCommand") {
		m_yPos = yPos;
		m_upToDown = true;
	}
	virtual ~DriveTrainWaitForFieldYCommand(){}

	void Initialize() {
		RigidTransform2D pose = m_driveTrain->GetObserver()->GetRobotPos(RobotController::GetFPGATime());
		m_upToDown = pose.getTranslation().getY() < m_yPos;
	}

	bool IsFinished() {
		RigidTransform2D pose = m_driveTrain->GetObserver()->GetRobotPos(RobotController::GetFPGATime());
		if(m_upToDown) {
			return pose.getTranslation().getY() > m_yPos;
		}
		else {
			return pose.getTranslation().getY() < m_yPos;
		}
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINWAITFORFIELDYCOMMAND_H_ */
