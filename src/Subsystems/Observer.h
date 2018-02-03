/*
 * Observer.h
 *
 *  Created on: Jan 9, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_SUBSYSTEMS_OBSERVER_H_
#define SRC_SUBSYSTEMS_OBSERVER_H_

#include "ctre/Phoenix.h"
#include "utils/RobotChains.h"
#include "utils/InterpolatingDouble.h"
#include "utils/InterpolatingMap.h"
#include "utils/RigidTransform2D.h"
#include "Kinematics.h"


class Observer {
public:
	Observer();
	virtual ~Observer();

	void ResetPose();

	void UpdateRobotPoseObservation(Rotation2D flAngle, RigidTransform2D::Delta flVelocity, Rotation2D frAngle,
			RigidTransform2D::Delta frVelocity, Rotation2D blAngle, RigidTransform2D::Delta blVelocity,
			Rotation2D brAngle, RigidTransform2D::Delta brVelocity, double timeStamp, Rotation2D deltaGyroYaw);
	RigidTransform2D GetRobotPos(double timestamp);
	void SetRobotPos(RigidTransform2D robotPos, double timestamp);
	RigidTransform2D GetLastRobotPose();

private:
	InterpolatingMap<InterpolatingDouble, RigidTransform2D> m_robotPos;
	Rotation2D m_oldGyroYaw;
	const double kFwdKinematicsWeight = 0.1;
	const double kGyroWeight = 0.9;
};

#endif /* SRC_SUBSYSTEMS_OBSERVER_H_ */
