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
#include "WPILib.h"

enum LineCrossed{
	NO_LINE = 0,
	NULL_LEFT = 1,
	NULL_RIGHT = 2,
	PLATFORM_LEFT = 3,
	PLATFORM_RIGHT = 4
};

class Observer {
public:
	Observer();
	virtual ~Observer();

	void ResetPose();
	void ResetPose(RigidTransform2D robotPose);
	void ResetPoseX(double x);
	void ResetPoseY(double y);

	void UpdateRobotPoseObservation(Rotation2D& flAngle, RigidTransform2D::Delta& flVelocity, Rotation2D& frAngle,
			RigidTransform2D::Delta& frVelocity, Rotation2D& blAngle, RigidTransform2D::Delta& blVelocity,
			Rotation2D& brAngle, RigidTransform2D::Delta& brVelocity, double timeStamp, Rotation2D& deltaGyroYaw);
	RigidTransform2D GetRobotPos(double timestamp);
	void SetRobotPos(RigidTransform2D robotPos, double timestamp);
	RigidTransform2D GetLastRobotPose();
	int LinePosCorrection();

private:
	InterpolatingMap<InterpolatingDouble, RigidTransform2D> m_robotPos;
	Rotation2D m_oldGyroYaw;
	const double kFwdKinematicsWeight = 0;
	const double kGyroWeight = 1;
	bool m_isFlLineDetected;
	bool m_isFrLineDetected;
	bool m_isBlLineDetected;
	bool m_isBrLineDetected;

	DigitalInput* m_flLineSensor;
	DigitalInput* m_frLineSensor;
	DigitalInput* m_blLineSensor;
	DigitalInput* m_brLineSensor;
};

#endif /* SRC_SUBSYSTEMS_OBSERVER_H_ */
