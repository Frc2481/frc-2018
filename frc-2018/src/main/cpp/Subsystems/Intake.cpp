/*
 * Intake.cpp
 *
 *  Created on: Jan 15, 2018
 *      Author: Team2481
 */

#include <Subsystems/Intake.h>
#include "RobotMap.h"
#include "frc/WPILib.h"

Intake::Intake() : frc::Subsystem("Intake") {
	m_rollerMotorLeft = new TalonSRX(INTAKE_LEFT_MOTOR);
	m_rollerMotorRight = new TalonSRX(INTAKE_RIGHT_MOTOR);
	m_clampSolenoid = new frc::Solenoid(INTAKE_CLAMP);

	m_rollerMotorLeft->ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);
	m_rollerMotorRight->ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);

	m_rollerMotorLeft->ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_Disabled, 0);
	m_rollerMotorRight->ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_Disabled, 0);

	m_rollerMotorLeft->SetInverted(true);
	m_rollerMotorRight->SetInverted(true);


//	m_rollerMotorLeft->ConfigPeakCurrentLimit(20, 0);
//	m_rollerMotorLeft->ConfigPeakCurrentDuration(0, 0);

	frc::SmartDashboard::PutNumber("intake current threshold", 0);
}

Intake::~Intake() {
}

bool Intake::HasCube() {
//	SmartDashboard::PutBoolean("roller left is limit closed", m_rollerMotorLeft->GetSensorCollection().IsFwdLimitSwitchClosed());
//	SmartDashboard::PutBoolean("roller right is limit closed", m_rollerMotorRight->GetSensorCollection().IsFwdLimitSwitchClosed());

	return m_rollerMotorLeft->GetSensorCollection().IsFwdLimitSwitchClosed() &&
			m_rollerMotorRight->GetSensorCollection().IsFwdLimitSwitchClosed();
}

bool Intake::IsLeftRollerOn() {
	return fabs(m_rollerMotorLeft->GetMotorOutputPercent()) > .01;
}

bool Intake::IsRightRollerOn() {
	return fabs(m_rollerMotorRight->GetMotorOutputPercent()) > .01;
}

void Intake::RollerOff() {
	m_rollerMotorLeft->Set(ControlMode::PercentOutput, 0);
	m_rollerMotorRight->Set(ControlMode::PercentOutput, 0);
}

bool Intake::IsClamped() {
	return !m_clampSolenoid->Get();
}

void Intake::OpenClamp() {
	m_clampSolenoid->Set(true);
}

void Intake::RollerLoad(double speed) {
	m_rollerMotorLeft->Set(ControlMode::PercentOutput, speed);
	m_rollerMotorRight->Set(ControlMode::PercentOutput, speed);
}

void Intake::RollerUnload(double speed) {
	m_rollerMotorLeft->Set(ControlMode::PercentOutput, -speed);
	m_rollerMotorRight->Set(ControlMode::PercentOutput, -speed);
}

void Intake::CloseClamp() {
	m_clampSolenoid->Set(false);
}

void Intake::Periodic() {
//	frc::SmartDashboard::PutNumber("IntakeRollerCurrentLeft", m_rollerMotorLeft->GetOutputCurrent());
//	frc::SmartDashboard::PutNumber("IntakeRollerCurrentRight", m_rollerMotorRight->GetOutputCurrent());
	frc::SmartDashboard::PutNumber("Has Cube", HasCube());
//	frc::SmartDashboard::PutNumber("average roller current", (m_rollerMotorLeft->GetOutputCurrent() + m_rollerMotorRight->GetOutputCurrent()) / 2.0);
}
