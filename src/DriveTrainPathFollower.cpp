/*
 * DriveTrainPathFollower.cpp
 *
 *  Created on: Mar 29, 2018
 *      Author: Team2481
 */

#include <DriveTrainPathFollower.h>
#include "CommandBase.h"

DriveTrainPathFollower::DriveTrainPathFollower(Observer *observer)
	: Looper(40000),
	  m_path(nullptr),
	  m_driveController(observer),
	  m_isFinished(false),
	  m_isFFFinished(false){

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
	uint64_t time = RobotController::GetFPGATime();
	uint64_t deltaTimestamp = time - m_prevTime;
	m_prevTime = time;

//	printf("DT: %ju\n", deltaTimestamp);

	CommandBase::m_driveTrain->PeriodicFast();

	if(!m_isFinished) {


		double logData[1+3+3+2+2+2+3];

		if(m_path != nullptr && ++m_currPoint != m_path->end() && !m_isFFFinished) {
			m_driveController.SetFieldTarget(*m_currPoint);
			logData[4] = m_currPoint->xPos;
			logData[5] = m_currPoint->yPos;
			logData[6] = m_currPoint->yaw;

			logData[7] = m_currPoint->xAccel;
			logData[8] = m_currPoint->yAccel;

			logData[9] = m_currPoint->xVel;
			logData[10] = m_currPoint->yVel;
		}
		else if (!m_isFFFinished) {
			m_isFFFinished = true;
			m_isFinished = true;
//			m_driveController.SetPosI(m_posI);
		} else {
			RigidTransform2D error = m_driveController.GetControllerError();
			if(fabs(error.getTranslation().getX()) < 1 && fabs(error.getTranslation().getY()) < 1) {
				m_isFinished = true;
				CommandBase::m_driveTrain->SetPreciseMode(false);
				CommandBase::m_driveTrain->Stop();
			}
		}


			RigidTransform2D robotPose = m_driveController.GetObserver()->GetLastRobotPose();
			logData[0] =  time;
			logData[1] = robotPose.getTranslation().getX();
			logData[2] = robotPose.getTranslation().getY();
			logData[3] = robotPose.getRotation().getDegrees();

			logData[11] = (robotPose.getTranslation().getX() - m_prevPosition.getTranslation().getX()) / (deltaTimestamp / 1000000.0);
			logData[12] = (robotPose.getTranslation().getY() - m_prevPosition.getTranslation().getY()) / (deltaTimestamp / 1000000.0);

			RigidTransform2D error = m_driveController.GetControllerError();
			logData[13] = error.getTranslation().getX();
			logData[14] = error.getTranslation().getY();
			logData[15] = error.getRotation().getDegrees();

			m_prevPosition = robotPose;

			SmartDashboard::PutNumberArray("path_log_data", llvm::ArrayRef<double>(logData, 16));

	//		SmartDashboard::PutNumber("PathX", m_currPoint->xPos);
	//		SmartDashboard::PutNumber("PathY", m_currPoint->yPos);
	//		SmartDashboard::PutNumber("PathYaw", m_currPoint->yaw);
	//
	//		SmartDashboard::PutNumber("PathX Accel", m_currPoint->xAccel);
	//		SmartDashboard::PutNumber("PathY Accel", m_currPoint->yAccel);

	//		SmartDashboard::PutNumber("PathX Vel", m_currPoint->xVel);
	//		SmartDashboard::PutNumber("PathY Vel", m_currPoint->yVel);

	//		SmartDashboard::PutNumber("Path Actual Heading", m_driveController.GetObserver()->GetLastRobotPose().getRotation().getDegrees());

			RigidTransform2D driveSignal = m_driveController.GetDriveControlSignal();
			CommandBase::m_driveTrain->Drive(driveSignal.getTranslation().getX(),
								driveSignal.getTranslation().getY(),
								driveSignal.getRotation().getDegrees());


	}
}

void DriveTrainPathFollower::FollowPath(Path2D* path) {
	m_path = path;
	m_currPoint = m_path->begin();
	m_isFinished = false;
	m_isFFFinished = false;
	CommandBase::m_driveTrain->SetPreciseMode(true);
	m_prevTime = RobotController::GetFPGATime();
	m_prevPosition = m_driveController.GetObserver()->GetLastRobotPose();
	m_driveController.SetPosI(0);
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
			Preferences::GetInstance()->GetDouble("POS_KD")),
			m_posI = Preferences::GetInstance()->GetDouble("POS KI");

	m_driveController.SetYawGains(
			Preferences::GetInstance()->GetDouble("YAW_KP"),
			Preferences::GetInstance()->GetDouble("YAW_KV"),
			Preferences::GetInstance()->GetDouble("YAW_KAP"),
			Preferences::GetInstance()->GetDouble("YAW_KAN"),
			Preferences::GetInstance()->GetDouble("YAW_KD"));
}
