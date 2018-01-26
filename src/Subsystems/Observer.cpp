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

void Observer::UpdatedRobotPositionObservation(Rotation2D flAngle, RigidTransform2D::Delta flVelocity, Rotation2D frAngle,
	RigidTransform2D::Delta frVelocity, Rotation2D blAngle, RigidTransform2D::Delta blVelocity,
	Rotation2D brAngle, RigidTransform2D::Delta brVelocity, double timestamp, Rotation2D deltaGyroYaw) {
	RigidTransform2D::Delta deltaRobotPos = Kinematics::SwerveForwardKinematics(flAngle, flVelocity, frAngle, frVelocity, blAngle, blVelocity, brAngle, brVelocity);

	RigidTransform2D oldRobotPos = GetLastRobotPos();

	// Complementary filter to combine gyro and forward kinematic angle deltas together.
	Rotation2D finalAngleDelta = Rotation2D::fromRadians((deltaRobotPos.GetTheta() * kFwdKinematicsWeight) + (deltaGyroYaw.getRadians() * kGyroWeight));
	Rotation2D newRobotAngle = oldRobotPos.getRotation().rotateBy(finalAngleDelta);

	Translation2D newRobotTranslation = oldRobotPos.getTranslation().
										translateBy(Translation2D(deltaRobotPos.GetX(),deltaRobotPos.GetY()).
										rotateBy(newRobotAngle.inverse()));

	RigidTransform2D robotPos(newRobotTranslation, newRobotAngle);
	SetRobotPos(robotPos, timestamp);

	SmartDashboard::PutNumber("delta robot angle gyro", deltaGyroYaw.getDegrees());
	SmartDashboard::PutNumber("delta robot angle kinematics", deltaRobotPos.GetTheta() * 180 / M_PI);
	SmartDashboard::PutNumber("delta robot x kinematics", deltaRobotPos.GetX());
	SmartDashboard::PutNumber("delta robot y kinematics", deltaRobotPos.GetY());
}

void Observer::AddGyroObservation(Rotation2D deltaGyroYaw, double timeStamp, double k) {
	RigidTransform2D oldRobotPos = GetLastRobotPos();
	Rotation2D newRobotAngle = oldRobotPos.getRotation().rotateBy(Rotation2D::fromDegrees(deltaGyroYaw.getDegrees() * k));
	RigidTransform2D robotPos(oldRobotPos.getTranslation(), newRobotAngle);
	SetRobotPos(robotPos, timeStamp + 0.0001);

	SmartDashboard::PutNumber("delta robot angle gyro", deltaGyroYaw.getDegrees());
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

RigidTransform2D Observer::GetLastRobotPos() {
	return m_robotPos.rbegin()->second;
}
