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

	void AddDriveTrainObservation(Rotation2D flAngle, RigidTransform2D::Delta flVelocity, Rotation2D frAngle,
			RigidTransform2D::Delta frVelocity, Rotation2D blAngle, RigidTransform2D::Delta blVelocity,
			Rotation2D brAngle, RigidTransform2D::Delta brVelocity, double timeStamp, double k);
	void AddGyroObservation(Rotation2D gyroAngle, double timeStamp, double k);
	RigidTransform2D GetRobotPos(double timestamp);
	void SetRobotPos(RigidTransform2D robotPos, double timestamp);
	RigidTransform2D GetLastRobotPos();

private:
	InterpolatingMap<InterpolatingDouble, RigidTransform2D> m_robotPos;


};

#endif /* SRC_SUBSYSTEMS_OBSERVER_H_ */
