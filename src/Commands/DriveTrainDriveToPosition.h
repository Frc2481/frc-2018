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

public:
	DriveTrainDriveToPosition() {
		Requires(m_driveTrain.get());
		m_driveController = m_driveTrain->GetDriveController();

		SmartDashboard::PutNumber("target x position", 0);
		SmartDashboard::PutNumber("target y position", 0);
		SmartDashboard::PutNumber("target heading", 0);

		SmartDashboard::PutNumber("x control signal", 0);
		SmartDashboard::PutNumber("y control signal", 0);
		SmartDashboard::PutNumber("yaw control signal", 0);

		SmartDashboard::PutNumber("Position Error to enable kiPos", 0);
		SmartDashboard::PutNumber("Yaw Error to enable kiYaw", 0);
		m_onTargetCounter = 0;
	}

	~DriveTrainDriveToPosition() {
	}

	void Initialize() {
		double xPos = SmartDashboard::GetNumber("target x position", 0);
		double yPos = SmartDashboard::GetNumber("target y position", 0);
		double yaw = SmartDashboard::GetNumber("target heading", 0);

		m_driveController->SetFieldTarget(RigidTransform2D(Translation2D(xPos, yPos), Rotation2D::fromDegrees(yaw)));
		m_driveController->EnableController();
		m_onTargetCounter = 0;
	}

	void Execute() {
		RigidTransform2D driveSignal = m_driveController->GetDriveControlSignal();
		m_driveTrain->Drive(driveSignal.getTranslation().getX(),
							driveSignal.getTranslation().getY(),
							driveSignal.getRotation().getDegrees());

		SmartDashboard::PutNumber("x control signal", driveSignal.getTranslation().getX());
		SmartDashboard::PutNumber("y control signal", driveSignal.getTranslation().getY());
		SmartDashboard::PutNumber("yaw control signal", driveSignal.getRotation().getDegrees());
	}

	void Interrupted() {
		End();
	}

	bool IsFinished() {
		return fabs(m_driveController->GetControllerError().getTranslation().norm()) < RobotParameters::kTolerancePos &&
			   fabs(m_driveController->GetControllerError().getRotation().getDegrees()) < RobotParameters::kToleranceHeading;
	}

	void End() {
		m_driveTrain->Drive(0, 0, 0);
		SmartDashboard::PutNumber("Drive to Position Duration", TimeSinceInitialized());
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINDRIVETOPOSITION_H_ */
