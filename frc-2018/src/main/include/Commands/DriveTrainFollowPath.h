/*
 * DriveTrainFollowPath.h
 *
 *  Created on: Jan 25, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINFOLLOWPATH_H_
#define SRC_COMMANDS_DRIVETRAINFOLLOWPATH_H_

#include "Components/DriveController.h"
#include "CommandBase.h"
#include "PIDController2481.h"
#include "utils/InterpolatingMap.h"
#include "utils/PathLoader.h"

class DriveTrainFollowPath : public CommandBase {
private:
	DriveController* m_driveController;
	bool m_skip;
	std::string m_filePath;

protected:
	Path2D& m_path;

public:
	DriveTrainFollowPath(std::string path) : m_path(m_pathManager->GetPath(path)){
		Requires(m_driveTrain.get());
		m_driveController = m_driveTrain->GetDriveController();
		m_skip = false;
		m_filePath = path;

		SmartDashboard::PutNumber("PathX", 0);
		SmartDashboard::PutNumber("PathY", 0);
		SmartDashboard::PutNumber("PathYaw", 0);
	}

	~DriveTrainFollowPath() {
	}

	void Initialize() {
		m_skip = false;
//		if(m_path.size() == 0) {
//			CommandGroup* parent = GetGroup();
//			if(parent != NULL) {
//				printf("skip follow path \n");
//				parent->Cancel();
//				m_skip = true;
//				return;
//			}
//		}
		m_driveController->EnableController();

		printf("running path %s start (%f %f %f) end (%f %f %f)\n",
				m_filePath.c_str(),
				m_path.begin()->second.getTranslation().getX(),
				m_path.begin()->second.getTranslation().getY(),
				m_path.begin()->second.getRotation().getDegrees(),
				m_path.end()->second.getTranslation().getX(),
				m_path.end()->second.getTranslation().getY(),
				m_path.end()->second.getRotation().getDegrees());
	}

	void Execute() {
		// Adding .25 to the time....Nolan can't figure out how to turn FeedForward so we have to use pure feedback.
		// by doing this the path following is awesome and trails the path by about 2 feet.  To avoid waiting for the
		// path to get 2 feet ahead of the current location we just start all the paths 0.25 seconds in because it works
		// and makes 3 cube autos great again.  This gives us a YUGE benefit!!!
		RigidTransform2D targetPos = m_path.getInterpolated(InterpolatingDouble(TimeSinceInitialized())); // + 0.4));
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
