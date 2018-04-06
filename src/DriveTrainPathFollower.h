/*
 * DriveTrainPathFollower.h
 *
 *  Created on: Mar 29, 2018
 *      Author: Team2481
 */

#ifndef SRC_DRIVETRAINPATHFOLLOWER_H_
#define SRC_DRIVETRAINPATHFOLLOWER_H_

#include "utils/Looper.h"
#include "utils/PathLoader.h"
#include "Components/DriveController.h"

class DriveTrainPathFollower : public Looper {
private:
	Path2D *m_path;
	Path2D::iterator m_currPoint;
	DriveController m_driveController;
	bool m_isFinished;

public:
	DriveTrainPathFollower(Observer *observer);
	virtual ~DriveTrainPathFollower();
	void OnStop();
	void OnStart();
	void OnLoop();
	void FollowPath(Path2D *path);
	bool IsFinished();
	void ReloadGains();
};

#endif /* SRC_DRIVETRAINPATHFOLLOWER_H_ */
