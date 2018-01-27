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

class DriveTrainDriveToPosition : public CommandBase {
private:
	DriveController* m_driveController;

public:
	DriveTrainDriveToPosition() {
		Requires(m_driveTrain.get());
		m_driveController = m_driveTrain->GetDriveController();

		SmartDashboard::PutNumber("target x position", 0);
		SmartDashboard::PutNumber("target y position", 0);
		SmartDashboard::PutNumber("target heading", 0);
		SmartDashboard::PutNumber("tolerance position", 0);
		SmartDashboard::PutNumber("tolerance heading", 0);

		SmartDashboard::PutNumber("kpPos", 0);
		SmartDashboard::PutNumber("kiPos", 0);
		SmartDashboard::PutNumber("kdPos", 0);
		SmartDashboard::PutNumber("kpYaw", 0);
		SmartDashboard::PutNumber("kiYaw", 0);
		SmartDashboard::PutNumber("kdYaw", 0);

		SmartDashboard::PutNumber("x control signal", 0);
		SmartDashboard::PutNumber("y control signal", 0);
		SmartDashboard::PutNumber("yaw control signal", 0);
	}

	~DriveTrainDriveToPosition() {
	}

	void Initialize() {
		double xPos = SmartDashboard::GetNumber("target x position", 0);
		double yPos = SmartDashboard::GetNumber("target y position", 0);
		double yaw = SmartDashboard::GetNumber("target heading", 0);
		double tolPos = SmartDashboard::GetNumber("tolerance position", 0);
		double tolYaw = SmartDashboard::GetNumber("tolerance heading", 0);

		double kpPos = SmartDashboard::GetNumber("kpPos", 0);
		double kiPos = SmartDashboard::GetNumber("kiPos", 0);
		double kdPos = SmartDashboard::GetNumber("kdPos", 0);
		double kpYaw = SmartDashboard::GetNumber("kpYaw", 0);
		double kiYaw = SmartDashboard::GetNumber("kiYaw", 0);
		double kdYaw = SmartDashboard::GetNumber("kdYaw", 0);

		m_driveController = m_driveTrain->GetDriveController();
		m_driveController->SetPIDGains(kpPos, kiPos, kdPos, kpYaw, kiYaw, kdYaw);
		m_driveController->SetFieldTarget(RigidTransform2D(Translation2D(xPos, yPos), Rotation2D::fromDegrees(yaw)),
										  RigidTransform2D(Translation2D(tolPos, tolPos), Rotation2D::fromDegrees(tolYaw)));
		m_driveController->EnableController();
	}

	void Execute() {
		RigidTransform2D driveSignal = m_driveController->GetDriveControlSignal();
		Translation2D translation = driveSignal.getTranslation();
		Rotation2D rotation = driveSignal.getRotation();

		SmartDashboard::PutNumber("x control signal", translation.getX());
		SmartDashboard::PutNumber("y control signal", translation.getY());
		SmartDashboard::PutNumber("yaw control signal", rotation.getDegrees());

		m_driveTrain->Drive(translation.getX(), translation.getY(), rotation.getDegrees());
	}

	void Interrupted() {
		End();
	}

	bool IsFinished() {
		return m_driveController->IsOnTarget();
	}

	void End() {
		m_driveTrain->Drive(0, 0, 0);
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINDRIVETOPOSITION_H_ */
