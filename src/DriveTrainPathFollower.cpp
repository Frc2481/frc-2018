/*
 * DriveTrainPathFollower.cpp
 *
 *  Created on: Mar 29, 2018
 *      Author: Team2481
 */

#include <DriveTrainPathFollower.h>
#include "CommandBase.h"

DriveTrainPathFollower::DriveTrainPathFollower(Observer *observer)
	: Looper(20000),
	  m_path(nullptr),
	  m_driveController(observer),
	  m_isFinished(false) {

	SmartDashboard::PutNumber("PathX", 0);
	SmartDashboard::PutNumber("PathY", 0);
	SmartDashboard::PutNumber("PathYaw", 0);

	SmartDashboard::PutNumber("PathX", 0);
	SmartDashboard::PutNumber("PathY", 0);
	SmartDashboard::PutNumber("PathYaw", 0);

	SmartDashboard::PutNumber("PathX Accel", 0);
	SmartDashboard::PutNumber("PathY Accel", 0);

	SmartDashboard::PutNumber("PathX Vel", 0);
	SmartDashboard::PutNumber("PathY Vel", 0);

	SmartDashboard::PutNumber("Path Actual Heading", 0);

}

DriveTrainPathFollower::~DriveTrainPathFollower() {
}

void DriveTrainPathFollower::OnStop() {
}

void DriveTrainPathFollower::OnStart() {
}

void DriveTrainPathFollower::OnLoop() {
	if(++m_currPoint != m_path->end() && !m_isFinished) {
		m_driveController.SetFieldTarget(*m_currPoint);

		SmartDashboard::PutNumber("PathX", m_currPoint->xPos);
		SmartDashboard::PutNumber("PathY", m_currPoint->yPos);
		SmartDashboard::PutNumber("PathYaw", m_currPoint->yaw);

		SmartDashboard::PutNumber("PathX Accel", m_currPoint->xAccel);
		SmartDashboard::PutNumber("PathY Accel", m_currPoint->yAccel);

		SmartDashboard::PutNumber("PathX Vel", m_currPoint->xVel);
		SmartDashboard::PutNumber("PathY Vel", m_currPoint->yVel);

		SmartDashboard::PutNumber("Path Actual Heading", m_driveController.GetObserver()->GetLastRobotPose().getRotation().getDegrees());

		RigidTransform2D driveSignal = m_driveController.GetDriveControlSignal();
		CommandBase::m_driveTrain->Drive(driveSignal.getTranslation().getX(),
							driveSignal.getTranslation().getY(),
							driveSignal.getRotation().getDegrees());


	}
	else {
		m_isFinished = true;
		CommandBase::m_driveTrain->SetPreciseMode(false);
	}
}

void DriveTrainPathFollower::FollowPath(Path2D* path) {
	m_path = path;
	m_currPoint = m_path->begin();
	m_isFinished = false;
	CommandBase::m_driveTrain->SetPreciseMode(true);
}

bool DriveTrainPathFollower::IsFinished() {
	return m_isFinished;
}

void DriveTrainPathFollower::ReloadGains() {
	m_driveController.SetPositionGains(
			Preferences::GetInstance()->GetDouble("POS_KP"),
			Preferences::GetInstance()->GetDouble("POS_KV"),
			Preferences::GetInstance()->GetDouble("POS_KAP"),
			Preferences::GetInstance()->GetDouble("POS_KAN"),
			Preferences::GetInstance()->GetDouble("POS_KD"));

	m_driveController.SetYawGains(
			Preferences::GetInstance()->GetDouble("YAW_KP"),
			Preferences::GetInstance()->GetDouble("YAW_KV"),
			Preferences::GetInstance()->GetDouble("YAW_KAP"),
			Preferences::GetInstance()->GetDouble("YAW_KAN"),
			Preferences::GetInstance()->GetDouble("YAW_KD"));
}
