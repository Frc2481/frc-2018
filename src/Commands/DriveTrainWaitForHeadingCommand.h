/*
 * DriveTrainWaitForHeadingCommand.h
 *
 *  Created on: Feb 22, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINWAITFORHEADINGCOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINWAITFORHEADINGCOMMAND_H_

#include "CommandBase.h"

class DriveTrainWaitForHeadingCommand : public CommandBase{
private:
	Rotation2D m_heading;
	bool m_clockwise;
public:
	DriveTrainWaitForHeadingCommand(Rotation2D heading) : CommandBase("DriveTrainWaitForHeadingCommand") {
		m_heading = heading;
		m_clockwise = false;
	}

	virtual ~DriveTrainWaitForHeadingCommand(){}

	void Initialize() {
		RigidTransform2D pose = m_driveTrain->GetObserver()->GetLastRobotPose();
		m_clockwise = pose.getRotation().rotateBy(m_heading.inverse()).getRadians() < 0;
	}

	bool IsFinished() {
		RigidTransform2D pose = m_driveTrain->GetObserver()->GetLastRobotPose();
		if(m_clockwise) {
			return pose.getRotation().rotateBy(m_heading.inverse()).getRadians() > 0;
		}
		else {
			return pose.getRotation().rotateBy(m_heading.inverse()).getRadians() < 0;
		}
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINWAITFORHEADINGCOMMAND_H_ */
