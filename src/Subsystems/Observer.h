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
	void AddDriveTrainObservation(Rotation2D flAngle, Translation2D flVelocity, Rotation2D frAngle,
			Translation2D frVelocity, Rotation2D blAngle, Translation2D blVelocity,
			Rotation2D brAngle, Translation2D brVelocity, double timeStamp);
	void AddGyroObservation(Rotation2D gyroAngle, double timeStamp);
	RigidTransform2D GetRobotPos(double timestamp);
	void SetRobotPos(RigidTransform2D robotPos, double timestamp);

private:
	InterpolatingMap<InterpolatingDouble, RigidTransform2D> m_robotPos;


};

#endif /* SRC_SUBSYSTEMS_OBSERVER_H_ */
