/*
 * Intake.cpp
 *
 *  Created on: Jan 15, 2018
 *      Author: Team2481
 */

#include <Subsystems/Intake.h>
#include "RobotMap.h"

Intake::Intake() {
	m_rollerMotor = new TalonSRX(INTAKE_MOTOR);
	m_clampSolenoid = new Solenoid(INTAKE_CLAMP);
}

Intake::~Intake() {
}

bool Intake::HasCube() {
	// TODO: Use some sensor.
	return false;
}

bool Intake::IsRollerOn() {
	return fabs(m_rollerMotor->GetMotorOutputPercent()) > .01;
}

void Intake::RollerOff() {
	m_rollerMotor->Set(ControlMode::PercentOutput, 0);
}

bool Intake::IsClamped() {
	return !m_clampSolenoid->Get();
}

void Intake::OpenClamp() {
	m_clampSolenoid->Set(true);
}

void Intake::RollerLoad() {
	m_rollerMotor->Set(ControlMode::PercentOutput, 1);
}

void Intake::RollerUnload() {
	m_rollerMotor->Set(ControlMode::PercentOutput, -1);
}

void Intake::CloseClamp() {
	m_clampSolenoid->Set(false);
}
