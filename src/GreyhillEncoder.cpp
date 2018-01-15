/*
 * GreyhillEncoder.cpp
 *
 *  Created on: Jul 10, 2017
 *      Author: Team2481
 */

#include <GreyhillEncoder.h>
#include <sstream>
#include <WPILib.h>

GreyhillEncoder::GreyhillEncoder(TalonSRX* talon, const std::string& name, int ticksPerRev, int inchesPerRev)
	: m_talon(talon), m_name(name), m_ticksPerRev(ticksPerRev), m_inchesPerRev(inchesPerRev) {

	m_talon->GetSelectedSensorPosition(QuadEncoder);
	m_talon->SetStatusFramePeriod(Status_2_Feedback0, 10, 0);
}

GreyhillEncoder::~GreyhillEncoder() {
	// TODO Auto-generated destructor stub
}

Translation2D GreyhillEncoder::GetRawDistance() const {
	return Translation2D(ConvertRotationsToInches(GetPosition()), 0);
}

Translation2D GreyhillEncoder::GetDistance() const {
	return GetRawDistance().translateBy(m_offset.inverse());
}

int GreyhillEncoder::GetPosition() const {
	return m_talon->GetSelectedSensorPosition(0);
}

void GreyhillEncoder::SetEncoderRaw(int ticks, int timeOut) {
	m_talon->SetSelectedSensorPosition(ticks, 0, timeOut);
}

double GreyhillEncoder::GetSpeed() const {
	return m_talon->GetSelectedSensorVelocity(0);
}

int GreyhillEncoder::GetRotations() const {
	return ConvertTicksToRotations(GetPosition());
}

double GreyhillEncoder::ConvertRotationsToInches(double rotations) const {
	return rotations * m_inchesPerRev;
}

double GreyhillEncoder::ConvertInchesToRotations(double inches) const {
	return inches / m_inchesPerRev;
}

int GreyhillEncoder::ConvertRotationsToTicks(double rotations) const {
	return rotations * m_ticksPerRev * 4; 								// four is to convert ticks to talon native units
}

double GreyhillEncoder::ConvertTicksToRotations(int ticks) const {
	return ticks / (m_ticksPerRev * 4); 								// four is to convert ticks to talon native units
}

void GreyhillEncoder::Reset() {
	m_offset = GetRawDistance();
}
