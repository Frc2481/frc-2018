/*
 * DriveController.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: Team2481
 */

#include "frc/WPILib.h"
#include "Components/DriveController.h"

DriveController::DriveController(Observer* observerObj) {
	m_observer = observerObj;

	m_positionXControlSource = new ObserverPIDSourceX(observerObj);
	m_positionYControlSource = new ObserverPIDSourceY(observerObj);
	m_positionYawControlSource = new ObserverPIDSourceYaw(observerObj);

	m_positionXControlSignal = new DriveControllerOutput();
	m_positionYControlSignal = new DriveControllerOutput();
	m_positionYawControlSignal = new DriveControllerOutput();

	m_positionXController = new frc::PIDController2481(RobotParameters::kpPos, RobotParameters::kiPos, RobotParameters::kdPos, RobotParameters::kfPos,
										m_positionXControlSource, m_positionXControlSignal, RobotParameters::PositionControllerPeriod);
	m_positionYController = new frc::PIDController2481(RobotParameters::kpPos, RobotParameters::kiPos, RobotParameters::kdPos, RobotParameters::kfPos,
										m_positionYControlSource, m_positionYControlSignal, RobotParameters::PositionControllerPeriod);
	m_positionYawController = new frc::PIDController2481(RobotParameters::kpYaw, RobotParameters::kiYaw, RobotParameters::kdYaw, RobotParameters::kfYaw,
										m_positionYawControlSource, m_positionYawControlSignal, RobotParameters::PositionControllerPeriod);

	m_positionYawController->SetInputRange(-180, 180);
	m_positionYawController->SetContinuous(true);

	m_positionXController->SetIZone(RobotParameters::kIZonePos);
	m_positionYController->SetIZone(RobotParameters::kIZonePos);
	m_positionYawController->SetIZone(RobotParameters::kIZoneYaw);
}

DriveController::~DriveController() {
}

void DriveController::SetFieldTarget(RigidTransform2D fieldTarget) {
	m_positionXController->SetSetpoint(fieldTarget.getTranslation().getX());
	m_positionYController->SetSetpoint(fieldTarget.getTranslation().getY());
	m_positionYawController->SetSetpoint(fieldTarget.getRotation().getDegrees());
}

void DriveController::SetRobotTarget(RigidTransform2D robotTarget) {
	//TODO transform from robot frame to field frame and call SetFieldTarget()
	m_positionXController->SetSetpoint(robotTarget.getTranslation().getX());
	m_positionYController->SetSetpoint(robotTarget.getTranslation().getY());
	m_positionYawController->SetSetpoint(robotTarget.getRotation().getDegrees());
}

bool DriveController::IsOnTarget() {
	bool onTarget = m_positionXController->OnTarget() && m_positionYController->OnTarget() && m_positionYawController->OnTarget();
	return onTarget;
}

RigidTransform2D DriveController::GetDriveControlSignal() {
	Translation2D controlSignalTranslation;
	Rotation2D controlSignalRotation;

	controlSignalTranslation.setX(m_positionXControlSignal->GetOutput());
	controlSignalTranslation.setY(m_positionYControlSignal->GetOutput());
	controlSignalRotation = Rotation2D::fromDegrees(m_positionYawControlSignal->GetOutput());

	//Convert drive signal from field frame to robot frame
	Rotation2D robotYaw = m_observer->GetLastRobotPose().getRotation();
	RigidTransform2D driveControlSignal(controlSignalTranslation.rotateBy(robotYaw.inverse()), controlSignalRotation);

	frc::SmartDashboard::PutNumber("Get Error X", m_positionXController->GetError());
	frc::SmartDashboard::PutNumber("Get Error Y", m_positionYController->GetError());
	frc::SmartDashboard::PutNumber("Get Error Yaw", m_positionYawController->GetError());

	return driveControlSignal;
}

RigidTransform2D DriveController::GetControllerError() {
	RigidTransform2D error;

	error.getTranslation().setX(m_positionXController->GetError());
	error.getTranslation().setY(m_positionYController->GetError());
	error.setRotation(Rotation2D::fromDegrees(m_positionYawController->GetError()));

	return error;
}

void DriveController::ResetController() {
	m_positionXController->Reset();
	m_positionYController->Reset();
	m_positionYawController->Reset();
}

void DriveController::EnableController() {
	ResetController();
	m_positionXController->Enable();
	m_positionYController->Enable();
	m_positionYawController->Enable();
}
