/*
 * Observer.h
 *
 *  Created on: Jan 9, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_SUBSYSTEMS_OBSERVER_H_
#define SRC_SUBSYSTEMS_OBSERVER_H_

#include "ctre/Phoenix.h"
#include "utils/Rotation2D.h"
#include "Translation2D.h"
#include "RobotChains.h"
#include "InterpolatingDouble.h"
#include "InterpolatingMap.h"
#include "RigidTransform2D.h"

class Observer {
public:
	Observer();
	virtual ~Observer();
	void AddDriveTrainObservation(RigidTransform2D robotCenterVel, double timeStamp);
	void AddGyroObservation(Rotation2D gyroAngle, double timeStamp);
	void TimeUpdateRobotState(double timestamp);

private:
	InterpolatingMap<InterpolatingDouble, Rotation2D> m_gyroAngleVelZ;
	InterpolatingMap<InterpolatingDouble, RigidTransform2D> m_driveTrainVel;
	InterpolatingMap<InterpolatingDouble, RigidTransform2D> m_robotPos;


};

#endif /* SRC_SUBSYSTEMS_OBSERVER_H_ */
