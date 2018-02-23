/*
 * DriveTrainDriveToPosition.h
 *
 *  Created on: Jan 25, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINDRIVETOPOSITION_H_
#define SRC_COMMANDS_DRIVETRAINDRIVETOPOSITION_H_

#include <Components/DriveController.h>
#include "CommandBase.h"
#include "../PIDController2481.h"

class DriveTrainDriveToPosition : public CommandBase {
private:
	DriveController* m_driveController;
	int m_onTargetCounter;
	double m_xPos;
	double m_yPos;
	double m_yaw;

public:
	DriveTrainDriveToPosition() {
		Requires(m_driveTrain.get());
		m_driveController = m_driveTrain->GetDriveController();

		SmartDashboard::PutNumber("target x position", 0);
		SmartDashboard::PutNumber("target y position", 0);
		SmartDashboard::PutNumber("target heading", 0);
		m_onTargetCounter = 0;
	}

	~DriveTrainDriveToPosition() {
	}

	void Initialize() {
		m_xPos = SmartDashboard::GetNumber("target x position", 0);
		m_yPos = SmartDashboard::GetNumber("target y position", 0);
		m_yaw = SmartDashboard::GetNumber("target heading", 0);

		m_driveController->SetFieldTarget(RigidTransform2D(Translation2D(m_xPos, m_yPos), Rotation2D::fromDegrees(m_yaw)));
		m_driveController->EnableController();
	}

	void Execute() {
		RigidTransform2D driveSignal = m_driveController->GetDriveControlSignal();
		m_driveTrain->Drive(driveSignal.getTranslation().getX(),
							driveSignal.getTranslation().getY(),
							driveSignal.getRotation().getDegrees());
	}

	void Interrupted() {
		End();
	}

	bool IsFinished() {

		RigidTransform2D lastPoint = RigidTransform2D(Translation2D(m_xPos, m_yPos), Rotation2D::fromDegrees(m_yaw));
		RigidTransform2D robotPose = m_driveTrain->GetObserver()->GetLastRobotPose();

		Translation2D errorTranslation = lastPoint.getTranslation().translateBy(robotPose.getTranslation().inverse());
		Rotation2D errorRotation = lastPoint.getRotation().rotateBy(robotPose.getRotation().inverse());

		return (fabs(errorTranslation.norm()) < RobotParameters::kTolerancePos) &&
			   (fabs(errorRotation.getDegrees()) < RobotParameters::kToleranceHeading);
	}

	void End() {
		m_driveTrain->Drive(0, 0, 0);
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINDRIVETOPOSITION_H_ */
