/*
 * DriveTrainDriveToPosition.h
 *
 *  Created on: Jan 25, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINDRIVEPATHCOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINDRIVEPATHCOMMAND_H_

#include <Components/DriveController.h>
#include "CommandBase.h"
#include "../PIDController2481.h"
#include <utils/InterpolatingMap.h>

class DriveTrainDrivePathCommand : public CommandBase {
private:
	DriveController* m_driveController;
	InterpolatingMap<InterpolatingDouble,RigidTransform2D> m_path;
public:
	DriveTrainDrivePathCommand() {
		Requires(m_driveTrain.get());
		m_driveController = m_driveTrain->GetDriveController();

		m_path.put(InterpolatingDouble(0), RigidTransform2D(Translation2D(0, 0), Rotation2D::fromDegrees(0)));
		m_path.put(InterpolatingDouble(3/2.0), RigidTransform2D(Translation2D(30, 0), Rotation2D::fromDegrees(0)));
		m_path.put(InterpolatingDouble(5.5/2.0), RigidTransform2D(Translation2D(35, 5), Rotation2D::fromDegrees(0)));
		m_path.put(InterpolatingDouble(6/2.0), RigidTransform2D(Translation2D(40, 10), Rotation2D::fromDegrees(0)));
		m_path.put(InterpolatingDouble(9/2.0), RigidTransform2D(Translation2D(40, 40), Rotation2D::fromDegrees(0)));
//		m_path.put(InterpolatingDouble(5.5), RigidTransform2D(Translation2D(30, 35), Rotation2D::fromDegrees(0)));

		SetTimeout(9/2.0);
	}

	~DriveTrainDrivePathCommand() {
	}

	void Initialize() {
		m_driveController->EnableController();
	}

	void Execute() {
		RigidTransform2D targetPos = m_path.getInterpolated(InterpolatingDouble(TimeSinceInitialized()));
		m_driveController->SetFieldTarget(targetPos,
										  RigidTransform2D(Translation2D(1, 1), Rotation2D::fromDegrees(1)));

		SmartDashboard::PutNumber("PathX",targetPos.getTranslation().getX());
		SmartDashboard::PutNumber("PathY",targetPos.getTranslation().getY());

		RigidTransform2D driveSignal = m_driveController->GetDriveControlSignal();
		m_driveTrain->Drive(driveSignal.getTranslation().getX(),
							driveSignal.getTranslation().getY(),
							driveSignal.getRotation().getDegrees());
	}

	void Interrupted() {
		End();
	}

	bool IsFinished() {
		return IsTimedOut();
	}

	void End() {
		m_driveTrain->Drive(0, 0, 0);
		SmartDashboard::PutNumber("Drive path Duration", TimeSinceInitialized());
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINDRIVEPATHCOMMAND_H_ */
