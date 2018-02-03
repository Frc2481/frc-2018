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

	m_positionYawController->SetInputRange(-180, 180);
	m_positionYawController->SetContinuous(true);

	SmartDashboard::PutNumber("RobotTarget X:", 0);
	SmartDashboard::PutNumber("RobotTarget Y:", 0);
	SmartDashboard::PutNumber("RobotTarget Yaw:", 0);


	SmartDashboard::PutData(m_positionXController);
	SmartDashboard::PutData(m_positionYController);
	SmartDashboard::PutData(m_positionYawController);
	// TODO make yaw controller continuous
}

DriveController::~DriveController() {
	// TODO Auto-generated destructor stub
}

void DriveController::SetFieldTarget(RigidTransform2D fieldTarget, RigidTransform2D absTolerance) {
	RigidTransform2D robotPoseInv = m_observer->GetLastRobotPose().inverse();

	RigidTransform2D robotTarget = fieldTarget;

	SmartDashboard::PutNumber("RobotTarget X:", robotTarget.getTranslation().getX());
	SmartDashboard::PutNumber("RobotTarget Y:", robotTarget.getTranslation().getY());
	SmartDashboard::PutNumber("RobotTarget Yaw:", robotTarget.getRotation().getDegrees());
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

	controlSignalTranslation.setX(m_positionXControlSignal->GetOutput());
	controlSignalTranslation.setY(m_positionYControlSignal->GetOutput());
	Rotation2D controlSignalRotation = Rotation2D::fromDegrees(m_positionYawControlSignal->GetOutput());

	SmartDashboard::PutNumber("Output Yaw Signal", m_positionYawControlSignal->GetOutput());

	m_positionYawController->IsEnabled();
	m_positionYawController->GetSetpoint();
	m_positionYawController->GetError();
	m_positionYawController->GetP();


	SmartDashboard::PutNumber("Get Setpoint X", m_positionXController->GetSetpoint());
	SmartDashboard::PutNumber("Get Error X", m_positionXController->GetError());

	SmartDashboard::PutNumber("Get Output", m_positionYControlSignal->GetOutput()); // saturates at 1
	SmartDashboard::PutBoolean("Is Enabled", m_positionYController->IsEnabled());
	SmartDashboard::PutNumber("Get Setpoint Y", m_positionYController->GetSetpoint());
	SmartDashboard::PutNumber("Get Error Y", m_positionYController->GetError());
	SmartDashboard::PutNumber("Get P", m_positionYController->GetP());

	SmartDashboard::PutNumber("Get Setpoint Yaw", m_positionYawController->GetSetpoint());
	SmartDashboard::PutNumber("Get Error Yaw", m_positionYawController->GetError());

	Rotation2D robotHeading = m_observer->GetLastRobotPose().getRotation();
	RigidTransform2D driveControlSignal(controlSignalTranslation.rotateBy(robotHeading.inverse()), controlSignalRotation);

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
