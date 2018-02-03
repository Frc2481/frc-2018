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
	int m_counter;

public:
	DriveTrainDriveToPosition() {
		Requires(m_driveTrain.get());
		m_driveController = m_driveTrain->GetDriveController();

		SmartDashboard::PutNumber("target x position", 0);
		SmartDashboard::PutNumber("target y position", 0);
		SmartDashboard::PutNumber("target heading", 0);
		SmartDashboard::PutNumber("tolerance position", RobotParameters::kTolerancePos);
		SmartDashboard::PutNumber("tolerance heading", RobotParameters::kToleranceHeading);

		SmartDashboard::PutNumber("x control signal", 0);
		SmartDashboard::PutNumber("y control signal", 0);
		SmartDashboard::PutNumber("yaw control signal", 0);

		m_counter = 0;
	}

	~DriveTrainDriveToPosition() {
	}

	void Initialize() {
		double xPos = SmartDashboard::GetNumber("target x position", 0);
		double yPos = SmartDashboard::GetNumber("target y position", 0);
		double yaw = SmartDashboard::GetNumber("target heading", 0);
		double tolPos = SmartDashboard::GetNumber("tolerance position", RobotParameters::kTolerancePos);
		double tolYaw = SmartDashboard::GetNumber("tolerance heading", RobotParameters::kToleranceHeading);

		m_driveController = m_driveTrain->GetDriveController();
//		m_driveController->SetPIDGains(kpPos, RobotParameters::kiPos, RobotParameters::kdPos,
//									   kpYaw, RobotParameters::kiYaw, RobotParameters::kdYaw);
		SmartDashboard::PutNumber("kpPosDebug2", m_driveController->m_positionYController->GetP());

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

		m_counter++;
		SmartDashboard::PutNumber("counter", m_counter);

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
