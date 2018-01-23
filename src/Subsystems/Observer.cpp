/*
 * Observer.cpp
 *
 *  Created on: Jan 9, 2018
 *      Author: FIRSTMentor
 */

#include <Subsystems/Observer.h>
#include "RobotParameters.h"
#include <cmath>
#include <algorithm>
#include "WPILib.h"

Observer::Observer() {

}

Observer::~Observer() {
	// TODO Auto-generated destructor stub
}

void Observer::AddDriveTrainObservation(Rotation2D flAngle, RigidTransform2D::Delta flVelocity, Rotation2D frAngle,
		RigidTransform2D::Delta frVelocity, Rotation2D blAngle, RigidTransform2D::Delta blVelocity,
		Rotation2D brAngle, RigidTransform2D::Delta brVelocity, double timestamp) {
	RigidTransform2D::Delta deltaRobotPos = Kinematics::SwerveForwardKinematics(flAngle, flVelocity, frAngle, frVelocity, blAngle, blVelocity, brAngle, brVelocity);

	RigidTransform2D oldRobotPos = GetRobotPos(timestamp - 20000); //m_robotPos.cbegin()->second; //ToDo: check timestamp
	Rotation2D newRobotAngle = oldRobotPos.getRotation().rotateBy(Rotation2D::fromRadians(deltaRobotPos.GetTheta()));

	Translation2D newRobotTranslation = oldRobotPos.getTranslation().translateBy(Translation2D(deltaRobotPos.GetX(),deltaRobotPos.GetY()));

	RigidTransform2D robotPos(newRobotTranslation, newRobotAngle);

	SetRobotPos(robotPos, timestamp);

	SmartDashboard::PutNumber("delta robot angle", deltaRobotPos.GetTheta());
	SmartDashboard::PutNumber("delta robot x", deltaRobotPos.GetX());
	SmartDashboard::PutNumber("delta robot y", deltaRobotPos.GetY());
}

void Observer::AddGyroObservation(Rotation2D gyroAngleVelZ, double timeStamp) {
	//SetRobotPos(robotPos, timestamp);
}

RigidTransform2D Observer::GetRobotPos(double timestamp) {
	return m_robotPos.getInterpolated(timestamp);
}

void Observer::SetRobotPos(RigidTransform2D robotPos, double timestamp) {
	m_robotPos.put(InterpolatingDouble(timestamp), robotPos);
}

void Observer::ResetPose() {
	m_robotPos.clear();
	SetRobotPos(RigidTransform2D(), RobotController::GetFPGATime());
}
