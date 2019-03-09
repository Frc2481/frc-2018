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
private:
	Path2D& m_path;

public:
	DriveTrainWaitForFinishedPathCommand(std::string path) : CommandBase("DriveTrainWaitForFinishedPathCommand"),
																		  m_path(m_pathManager->GetPath(path)) {

	}
	virtual ~DriveTrainWaitForFinishedPathCommand(){}

	void Initialize() {
		RigidTransform2D pose = m_driveTrain->GetObserver()->GetRobotPos(RobotController::GetFPGATime());
//		m_path.rend()->second;
	}

	bool IsFinished() {
		RigidTransform2D lastPoint = m_path.rbegin()->second;
		RigidTransform2D robotPose = m_driveTrain->GetObserver()->GetLastRobotPose();

		Translation2D errorTranslation = lastPoint.getTranslation().translateBy(robotPose.getTranslation().inverse());
		Rotation2D errorRotation = lastPoint.getRotation().rotateBy(robotPose.getRotation().inverse());

		return ((fabs(errorTranslation.norm()) < RobotParameters::kTolerancePos) &&
			   (fabs(errorRotation.getDegrees()) < RobotParameters::kToleranceHeading));
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINWAITFORFINISHEDPATHCOMMAND_H_ */
