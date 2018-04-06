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

Observer::Observer() : m_robotPos(5){

}

Observer::~Observer() {
	// TODO Auto-generated destructor stub
}

void Observer::UpdateRobotPoseObservation(Rotation2D& flAngle, RigidTransform2D::Delta& flVelocity, Rotation2D& frAngle,
	RigidTransform2D::Delta& frVelocity, Rotation2D& blAngle, RigidTransform2D::Delta& blVelocity,
	Rotation2D& brAngle, RigidTransform2D::Delta& brVelocity, double timestamp, Rotation2D& deltaGyroYaw) {
	RigidTransform2D::Delta deltaRobotPos = Kinematics::SwerveForwardKinematics(flAngle, flVelocity, frAngle, frVelocity, blAngle, blVelocity, brAngle, brVelocity, deltaGyroYaw);

	RigidTransform2D oldRobotPos = GetLastRobotPose();

	// Complementary filter to combine gyro and forward kinematic angle deltas together.
	Rotation2D finalAngleDelta = Rotation2D::fromRadians((deltaRobotPos.GetTheta() * kFwdKinematicsWeight) + (deltaGyroYaw.getRadians() * kGyroWeight));
	Rotation2D newRobotAngle = oldRobotPos.getRotation().rotateBy(finalAngleDelta);

	Translation2D newRobotTranslation = oldRobotPos.getTranslation().translateBy(Translation2D(deltaRobotPos.GetX(), deltaRobotPos.GetY()).rotateBy(newRobotAngle));

	RigidTransform2D robotPos(newRobotTranslation, newRobotAngle);
	SetRobotPos(robotPos, timestamp);

//	SmartDashboard::PutNumber("delta robot angle gyro", deltaGyroYaw.getDegrees());
//	SmartDashboard::PutNumber("delta robot angle kinematics", deltaRobotPos.GetTheta() * 180 / M_PI);
//	SmartDashboard::PutNumber("delta robot x kinematics", deltaRobotPos.GetX());
//	SmartDashboard::PutNumber("delta robot y kinematics", deltaRobotPos.GetY());
}

RigidTransform2D Observer::GetRobotPos(double timestamp) {
	return m_robotPos.getInterpolated(timestamp);
}

void Observer::SetRobotPos(RigidTransform2D robotPos, double timestamp) {
	m_robotPos.put(InterpolatingDouble(timestamp), robotPos);
}

void Observer::ResetPose(RigidTransform2D robotPose) {
	m_robotPos.clear();
	SetRobotPos(robotPose, RobotController::GetFPGATime() - 37);  // This number DOES NOT MATTER
	SetRobotPos(robotPose, RobotController::GetFPGATime());
}

void Observer::ResetPose() {
	m_robotPos.clear();
	Translation2D zeroTranslation = Translation2D(0, 0);
	Rotation2D zeroRotation = Rotation2D(1, 0, true);
	SetRobotPos(RigidTransform2D(zeroTranslation, zeroRotation), RobotController::GetFPGATime());
}

RigidTransform2D Observer::GetLastRobotPose() {
	return m_robotPos.rbegin()->second;
}
