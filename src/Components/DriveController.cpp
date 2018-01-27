/*
 * DriveController.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: Team2481
 */

#include "Components/DriveController.h"

DriveController::DriveController(Observer* observerObj) {
	// TODO Auto-generated constructor stub
	m_observer = observerObj;

	m_positionXControlSource = new ObserverPIDSourceX(observerObj);
	m_positionYControlSource = new ObserverPIDSourceY(observerObj);
	m_positionYawControlSource = new ObserverPIDSourceYaw(observerObj);

	m_positionXControlSignal = new DriveControllerOutput();
	m_positionYControlSignal = new DriveControllerOutput();
	m_positionYawControlSignal = new DriveControllerOutput();

	m_positionXController = new PIDController(RobotParameters::kpPos, RobotParameters::kiPos, RobotParameters::kdPos,
										m_positionXControlSource, m_positionXControlSignal, RobotParameters::PositionControllerPeriod);
	m_positionYController = new PIDController(RobotParameters::kpPos, RobotParameters::kiPos, RobotParameters::kdPos,
										m_positionYControlSource, m_positionYControlSignal, RobotParameters::PositionControllerPeriod);
	m_positionYawController = new PIDController(RobotParameters::kpYaw, RobotParameters::kiYaw, RobotParameters::kdYaw,
										m_positionYawControlSource, m_positionYawControlSignal, RobotParameters::PositionControllerPeriod);

	SmartDashboard::PutNumber("RobotTarget X:", 0);
	SmartDashboard::PutNumber("RobotTarget Y:", 0);
	SmartDashboard::PutNumber("RobotTarget Yaw:", 0);
}

DriveController::~DriveController() {
	// TODO Auto-generated destructor stub
}

void DriveController::SetFieldTarget(RigidTransform2D fieldTarget, RigidTransform2D absTolerance) {
	RigidTransform2D robotTarget = fieldTarget.transformBy(m_observer->GetLastRobotPos().inverse());
	SmartDashboard::PutNumber("RobotTarget X:", m_observer->GetLastRobotPos().inverse().getTranslation().getX());
	SmartDashboard::PutNumber("RobotTarget Y:", m_observer->GetLastRobotPos().inverse().getTranslation().getY());
	SmartDashboard::PutNumber("RobotTarget Yaw:", m_observer->GetLastRobotPos().inverse().getRotation().getDegrees());
	SetRobotTarget(robotTarget, absTolerance);
}

void DriveController::SetRobotTarget(RigidTransform2D robotTarget, RigidTransform2D absTolerance) {
	m_positionXController->SetSetpoint(robotTarget.getTranslation().getX());
	m_positionYController->SetSetpoint(robotTarget.getTranslation().getY());
	m_positionYawController->SetSetpoint(robotTarget.getRotation().getDegrees());

	m_positionXController->SetAbsoluteTolerance(absTolerance.getTranslation().getX());
	m_positionYController->SetAbsoluteTolerance(absTolerance.getTranslation().getY());
	m_positionYawController->SetAbsoluteTolerance(absTolerance.getRotation().getDegrees());
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
	controlSignalRotation.fromDegrees(m_positionYawControlSignal->GetOutput());

	RigidTransform2D driveControlSignal(controlSignalTranslation, controlSignalRotation);
	return driveControlSignal;
}

void DriveController::SetPIDGains(double kpPos, double kiPos, double kdPos,
								  double kpYaw, double kiYaw, double kdYaw) {
	m_positionXController->SetPID(kpPos, kiPos, kdPos);
	m_positionYController->SetPID(kpPos, kiPos, kdPos);
	m_positionYawController->SetPID(kpYaw, kiYaw, kdYaw);
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
