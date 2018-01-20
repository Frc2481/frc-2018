/*
 * Observer.cpp
 *
 *  Created on: Jan 9, 2018
 *      Author: FIRSTMentor
 */

#include <Subsystems/Observer.h>
#include "Components/SwerveModuleV2Constants.h"
#include <cmath>
#include <algorithm>
#include "WPILib.h"

Observer::Observer() {

}

Observer::~Observer() {
	// TODO Auto-generated destructor stub
}

void Observer::AddDriveTrainObservation(Rotation2D flAngle, Translation2D flVelocity, Rotation2D frAngle,
		Translation2D frVelocity, Rotation2D blAngle, Translation2D blVelocity,
		Rotation2D brAngle, Translation2D brVelocity, double timestamp) {
	RigidTransform2D deltaRobotPos = Kinematics::SwerveForwardKinematics(flAngle, flVelocity, frAngle, frVelocity, blAngle, blVelocity, brAngle, brVelocity);

	RigidTransform2D oldRobotPos = GetRobotPos(timestamp - 20000);//m_robotPos.cbegin()->second;
	Rotation2D newRobotAngle = oldRobotPos.getRotation().rotateBy(deltaRobotPos.getRotation());

	SmartDashboard::PutNumber("delta robot angle", deltaRobotPos.getRotation().getDegrees());
	SmartDashboard::PutNumber("delta robot x", deltaRobotPos.getTranslation().getX());
	SmartDashboard::PutNumber("delta robot y", deltaRobotPos.getTranslation().getY());

	SmartDashboard::PutNumber("fabs fl delta robot x", fabs(deltaRobotPos.getTranslation().getX()) - fabs(flVelocity.getX()));
	SmartDashboard::PutNumber("fabs fr delta robot x", fabs(deltaRobotPos.getTranslation().getX()) - fabs(frVelocity.getX()));
	SmartDashboard::PutNumber("fabs bl delta robot x", fabs(deltaRobotPos.getTranslation().getX()) - fabs(blVelocity.getX()));
	SmartDashboard::PutNumber("fabs br delta robot x", fabs(deltaRobotPos.getTranslation().getX()) - fabs(brVelocity.getX()));

	SmartDashboard::PutNumber("avg delta robot x", (fabs(deltaRobotPos.getTranslation().getX()) - fabs(flVelocity.getX()) +
													fabs(deltaRobotPos.getTranslation().getX()) - fabs(frVelocity.getX()) +
													fabs(deltaRobotPos.getTranslation().getX()) - fabs(blVelocity.getX()) +
													fabs(deltaRobotPos.getTranslation().getX()) - fabs(brVelocity.getX())) / 4.0);

	SmartDashboard::PutNumber("fabs fl delta robot y", fabs(deltaRobotPos.getTranslation().getY()) - fabs(flVelocity.getX()));
	SmartDashboard::PutNumber("fabs fr delta robot y", fabs(deltaRobotPos.getTranslation().getY()) - fabs(frVelocity.getX()));
	SmartDashboard::PutNumber("fabs bl delta robot y", fabs(deltaRobotPos.getTranslation().getY()) - fabs(blVelocity.getX()));
	SmartDashboard::PutNumber("fabs br delta robot y", fabs(deltaRobotPos.getTranslation().getY()) - fabs(brVelocity.getX()));

	SmartDashboard::PutNumber("avg delta robot y", (fabs(deltaRobotPos.getTranslation().getY()) - fabs(flVelocity.getX()) +
													fabs(deltaRobotPos.getTranslation().getY()) - fabs(frVelocity.getX()) +
													fabs(deltaRobotPos.getTranslation().getY()) - fabs(blVelocity.getX()) +
													fabs(deltaRobotPos.getTranslation().getY()) - fabs(brVelocity.getX())) / 4.0);

	Translation2D newRobotTranslation = oldRobotPos.getTranslation().translateBy(deltaRobotPos.getTranslation().rotateBy(newRobotAngle));

	RigidTransform2D robotPos(newRobotTranslation, newRobotAngle);

	SetRobotPos(robotPos, timestamp);
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
