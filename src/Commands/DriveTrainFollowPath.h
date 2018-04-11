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
	bool m_skip;
	std::string m_filePath;

protected:
	Path2D& m_path;

public:
	DriveTrainFollowPath(std::string path) : m_path(m_pathManager->GetPath(path)){
		Requires(m_driveTrain.get());
		m_skip = false;
		m_filePath = path;
	}

	~DriveTrainFollowPath() {
	}

	void Initialize() {
		m_pathFollower->ReloadGains();

		m_skip = m_path.empty();

		m_pathFollower->FollowPath(&m_path);
		m_pathFollower->SetActive(true);

		printf("running path %s\n", m_filePath.c_str());
	}

	void Execute() {
		// Adding .25 to the time....Nolan can't figure out how to turn FeedForward so we have to use pure feedback.
		// by doing this the path following is awesome and trails the path by about 2 feet.  To avoid waiting for the
		// path to get 2 feet ahead of the current location we just start all the paths 0.25 seconds in because it works
		// and makes 3 cube autos great again.
		//UPDATE: Nolan is letting us use feed forward. This gives us a YUGE benefit!!!
	}

	bool IsFinished() {
		return m_pathFollower->IsFinished();
	}

	void End() {
		m_pathFollower->SetActive(false);
		m_driveTrain->Drive(0, 0, 0);

		//logging error at end of path
		PathPoint2D &lastPoint = *m_path.rbegin();
		RigidTransform2D robotPose = m_driveTrain->GetObserver()->GetLastRobotPose();

		Translation2D errorTranslation(lastPoint.xPos - robotPose.getTranslation().getX(), lastPoint.yPos - robotPose.getTranslation().getY());
		Rotation2D errorRotation = Rotation2D::fromDegrees(lastPoint.yaw).rotateBy(robotPose.getRotation().inverse());

		printf("path finished xError: %f yError: %f headingError: %f duration: %f/n",
				errorTranslation.getX(), errorTranslation.getY(), errorRotation.getDegrees(), TimeSinceInitialized());
	}

	void Interrupted() {
		End();
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINFOLLOWPATH_H_ */
