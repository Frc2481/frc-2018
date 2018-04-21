/*
 * DriveController.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: Team2481
 */

#include "Components/DriveController.h"

DriveController::DriveController(Observer* observerObj) {
	m_observer = observerObj;

	m_positionXController = new PVAController(RobotParameters::kpDrivePos, RobotParameters::kvDrivePos, RobotParameters::kapDrivePos, RobotParameters::kanDrivePos, RobotParameters::kdDrivePos);
	m_positionYController = new PVAController(RobotParameters::kpDrivePos, RobotParameters::kvDrivePos, RobotParameters::kapDrivePos, RobotParameters::kanDrivePos, RobotParameters::kdDrivePos);
	m_positionYawController = new PVAController(RobotParameters::kpDriveYaw, RobotParameters::kvDriveYaw, RobotParameters::kaDriveYaw, 0, RobotParameters::kdDriveYaw);

	m_positionYawController->SetPositionLimits(-180, 180);
	m_positionYawController->SetContinuous(true);
}

DriveController::~DriveController() {
}

void DriveController::SetFieldTarget(PathPoint2D &fieldTarget) {
	m_positionXController->SetTarget(fieldTarget.xPos, fieldTarget.xVel, fieldTarget.xAccel);
	m_positionYController->SetTarget(fieldTarget.yPos, fieldTarget.yVel, fieldTarget.yAccel);
	m_positionYawController->SetTarget(fieldTarget.yaw, fieldTarget.yawVel, fieldTarget.yawAccel);
}

void DriveController::SetRobotTarget(PathPoint2D &robotTarget) {
	//TODO transform from robot frame to field frame and call SetFieldTarget()
	m_positionXController->SetTarget(robotTarget.xPos, robotTarget.xVel, robotTarget.xAccel);
	m_positionYController->SetTarget(robotTarget.yPos, robotTarget.yVel, robotTarget.yAccel);
	m_positionYawController->SetTarget(robotTarget.yaw, 0, 0);
}

RigidTransform2D DriveController::GetDriveControlSignal() {
	m_positionXController->SetActualPosition(m_observer->GetLastRobotPose().getTranslation().getX());
	m_positionYController->SetActualPosition(m_observer->GetLastRobotPose().getTranslation().getY());
	m_positionYawController->SetActualPosition(m_observer->GetLastRobotPose().getRotation().getDegrees());

	double xOutput = m_positionXController->CalculateVelocityControlSignal();
	double yOutput = m_positionYController->CalculateVelocityControlSignal();
	double yawOutput = std::max(-0.5, std::min(m_positionYawController->CalculateVelocityControlSignal(), 0.5));

	Translation2D controlSignalTranslation(xOutput, yOutput);
	Rotation2D controlSignalRotation = Rotation2D::fromDegrees(yawOutput);

	//Convert drive signal from field frame to robot frame
	Rotation2D robotYaw = m_observer->GetLastRobotPose().getRotation();
	RigidTransform2D driveControlSignal(controlSignalTranslation.rotateBy(robotYaw.inverse()), controlSignalRotation);

//	if(fabs(m_positionXController->GetError()) < 50) {
//		SmartDashboard::PutNumber("Get Error X", m_positionXController->GetError());
//	}
//	if(fabs(m_positionYController->GetError()) < 50) {
//		SmartDashboard::PutNumber("Get Error Y", m_positionYController->GetError());
//	}
//	if(fabs(m_positionYawController->GetError()) < 50) {
//		SmartDashboard::PutNumber("Get Error Yaw", m_positionYawController->GetError());
//	}

	return driveControlSignal;
}

RigidTransform2D DriveController::GetControllerError() {
	RigidTransform2D error;

	error.getTranslation().setX(m_positionXController->GetError());
	error.getTranslation().setY(m_positionYController->GetError());
	error.setRotation(Rotation2D::fromDegrees(m_positionYawController->GetError()));

	return error;
}

Observer* DriveController::GetObserver() {
	return m_observer;
}

void DriveController::ResetController() {
	m_positionXController->Reset();
	m_positionYController->Reset();
	m_positionYawController->Reset();
}

void DriveController::SetPositionGains(double kp, double kv, double kap, double kan, double kd) {
	m_positionXController->SetGains(kp, kv, kap, kan, kd);
	m_positionYController->SetGains(kp, kv, kap, kan, kd);
}

void DriveController::SetYawGains(double kp, double kv, double kap, double kan, double kd) {
	m_positionYawController->SetGains(kp, kv, kap, kan, kd);
}

void DriveController::SetPosI(double ki) {
	m_positionXController->SetIGain(ki);
	m_positionYController->SetIGain(ki);
}
