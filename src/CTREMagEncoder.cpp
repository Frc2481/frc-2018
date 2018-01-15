/*
 * CTREMagEncoder.cpp
 *
 *  Created on: Jul 3, 2017
 *      Author: Team2481
 */

#include <sstream>
#include <CTREMagEncoder.h>
#include <WPILib.h>

CTREMagEncoder::CTREMagEncoder(TalonSRX* talon, const std::string& name)
	: m_talon(talon), m_name(name) {

	std::stringstream ss;
	ss << "ENCODER_OFFSET_" << name;
	m_calibrationKey = ss.str();

	m_offset = Rotation2D::fromDegrees(Preferences::GetInstance()->GetDouble(m_calibrationKey));

	m_talon->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Absolute, 0, 0);
	m_talon->SetSensorPhase(true);
	m_talon->SetStatusFramePeriod(Status_2_Feedback0, 10, 0);
}

CTREMagEncoder::~CTREMagEncoder() {
	// TODO Auto-generated destructor stub
}

Rotation2D CTREMagEncoder::GetRawAngle() const {
	return Rotation2D::fromRadians(GetEncoderTicks() / 4096.0 * 2 * M_PI);

}

Rotation2D CTREMagEncoder::GetAngle() const {
	return m_offset.rotateBy(GetRawAngle());
}

int CTREMagEncoder::GetRotations() const {
	SmartDashboard::PutNumber("Rotations", GetEncoderTicks(true)/4096);
	return GetEncoderTicks(true) / 4096;
}

int CTREMagEncoder::GetEncoderTicks(bool overflow) const {
	int ticks = m_talon->GetSelectedSensorPosition(0);

	if (!overflow) {
		ticks &= 0xFFF;
	}
	return ticks;
}

void CTREMagEncoder::Calibrate() {
	m_offset = GetRawAngle().inverse();
	Preferences::GetInstance()->PutDouble(m_calibrationKey, m_offset.getDegrees());
}

int CTREMagEncoder::ConvertAngleToSetpoint(Rotation2D targetAngle) {
	Rotation2D angle = m_offset.inverse().rotateBy(targetAngle);

	int ticks = ConvertAngleToEncoderTicks(angle);
	int encoderTicks = GetEncoderTicks(true);

	ticks += GetRotations() * 4096;

	int error = encoderTicks - ticks;

	if (error < -2048) {
		ticks -= 4096;
	}
	else if (error > 2048) {
		ticks += 4096;
	}

	return ticks;
}

int CTREMagEncoder::ConvertAngleToEncoderTicks(Rotation2D angle) {
	double degrees = angle.getDegrees();
	return degrees / 360.0 * 4096;
}

void CTREMagEncoder::SetEncoderRaw(int ticks) {
	m_talon->SetSelectedSensorPosition(ticks, 0, 0);
}
