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
#include <Components/PathLoader.h>

class DriveTrainDrivePathCommand : public CommandBase {
private:
	DriveController* m_driveController;

protected:
	Path2D m_path;

public:
	DriveTrainDrivePathCommand() {
		Requires(m_driveTrain.get());
		m_driveController = m_driveTrain->GetDriveController();
	}

	~DriveTrainDrivePathCommand() {
	}

	void Initialize() {
		m_driveController->EnableController();
	}

	void Execute() {
		RigidTransform2D targetPos = m_path.getInterpolated(InterpolatingDouble(TimeSinceInitialized()));
		m_driveController->SetFieldTarget(targetPos);

		SmartDashboard::PutNumber("PathX",targetPos.getTranslation().getX());
		SmartDashboard::PutNumber("PathY",targetPos.getTranslation().getY());
		SmartDashboard::PutNumber("PathYaw",targetPos.getRotation().getDegrees());

		RigidTransform2D driveSignal = m_driveController->GetDriveControlSignal();
		m_driveTrain->Drive(driveSignal.getTranslation().getX(),
							driveSignal.getTranslation().getY(),
							driveSignal.getRotation().getDegrees());
	}

	void Interrupted() {
		End();
	}

	bool IsFinished() {
		RigidTransform2D lastPoint = m_path.rbegin()->second;
		RigidTransform2D robotPose = m_driveTrain->GetObserver()->GetLastRobotPose();

		Translation2D errorTranslation = lastPoint.getTranslation().translateBy(robotPose.getTranslation().inverse());
		Rotation2D errorRotation = lastPoint.getRotation().rotateBy(robotPose.getRotation().inverse());

		return (fabs(errorTranslation.norm()) < RobotParameters::kTolerancePos) &&
			   (fabs(errorRotation.getDegrees()) < RobotParameters::kToleranceHeading);
	}

	void End() {
		m_driveTrain->Drive(0, 0, 0);
		SmartDashboard::PutNumber("Drive path Duration", TimeSinceInitialized());
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINDRIVEPATHCOMMAND_H_ */
