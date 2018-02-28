/*
 * DriveTrainFollowPath.h
 *
 *  Created on: Jan 25, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINFOLLOWPATH_H_
#define SRC_COMMANDS_DRIVETRAINFOLLOWPATH_H_

#include <Components/DriveController.h>
#include "CommandBase.h"
#include "../PIDController2481.h"
#include <utils/InterpolatingMap.h>
#include <utils/PathLoader.h>

class DriveTrainFollowPath : public CommandBase {
private:
	DriveController* m_driveController;
	bool m_skip;

protected:
	Path2D& m_path;

public:
	DriveTrainFollowPath(std::string path) : m_path(m_pathManager->GetPath(path)){
		Requires(m_driveTrain.get());
		m_driveController = m_driveTrain->GetDriveController();
		m_skip = false;

		SmartDashboard::PutNumber("PathX", 0);
		SmartDashboard::PutNumber("PathY", 0);
		SmartDashboard::PutNumber("PathYaw", 0);
	}

	~DriveTrainFollowPath() {
	}

	void Initialize() {
		m_skip = false;
		if(m_path.size() == 0) {
			CommandGroup* parent = GetGroup();
			if(parent != NULL) {
				parent->Cancel();
				m_skip = true;
				return;
			}
		}
		m_driveController->EnableController();

		printf("running path start pos x", m_path.begin()->second.getTranslation().getX());
		printf("running path start pos y", m_path.begin()->second.getTranslation().getY());

		printf("running path end pos x", m_path.end()->second.getTranslation().getX());
		printf("running path end pos y", m_path.end()->second.getTranslation().getY());
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

		return ((fabs(errorTranslation.norm()) < RobotParameters::kTolerancePos) &&
			      (fabs(errorRotation.getDegrees()) < RobotParameters::kToleranceHeading)) ||
				    (m_path.rbegin()->first.m_value + 2 < TimeSinceInitialized()) ||
				    m_skip;
	}

	void End() {
		m_driveTrain->Drive(0, 0, 0);
		SmartDashboard::PutNumber("Drive path Duration", TimeSinceInitialized());
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINFOLLOWPATH_H_ */
