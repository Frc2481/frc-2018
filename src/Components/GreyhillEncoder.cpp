/*
 * GreyhillEncoder.cpp
 *
 *  Created on: Jul 10, 2017
 *      Author: Team2481
 */

#include <Components/GreyhillEncoder.h>
#include <sstream>
#include <WPILib.h>

GreyhillEncoder::GreyhillEncoder(TalonSRX* talon, const std::string& name, int ticksPerRev, double inchesPerWheelRev, double encoderRevPerWheelRevLowGear, double encoderRevPerWheelRevHighGear)
	: m_talon(talon), m_name(name), m_ticksPerRev(ticksPerRev), m_inchesPerWheelRev(inchesPerWheelRev),
	  m_encoderRevPerWheelRevLowGear(encoderRevPerWheelRevLowGear), m_encoderRevPerWheelRevHighGear(encoderRevPerWheelRevHighGear) {

	m_talon->GetSelectedSensorPosition(QuadEncoder);
	m_talon->SetStatusFramePeriod(Status_2_Feedback0, 10, 0);
}

GreyhillEncoder::~GreyhillEncoder() {
	// TODO Auto-generated destructor stub
}

Translation2D GreyhillEncoder::GetRawDistance() const {
	// convert ticks to encoder rotations
	double encoderRotations = ConvertEncoderTicksToEncoderRotations(GetEncoderTicks());

	// convert encoder rotations to wheel rotations
	double wheelRotations = ConvertEncoderRotationsToWheelRotations(encoderRotations);

	// convert wheel rotations to distance
	return Translation2D(ConvertWheelRotationsToDistance(wheelRotations), 0);
}

Translation2D GreyhillEncoder::GetDistance() const {
	return GetRawDistance().translateBy(m_offset.inverse());
}

int GreyhillEncoder::GetEncoderTicks() const {
	return m_talon->GetSelectedSensorPosition(0);
}

void GreyhillEncoder::SetEncoderTicks(int ticks, int timeOut) {
	m_talon->SetSelectedSensorPosition(ticks, 0, timeOut);
}

double GreyhillEncoder::GetEncoderSpeed() const {
	return m_talon->GetSelectedSensorVelocity(0);
}

double GreyhillEncoder::ConvertEncoderRotationsToWheelRotations(double rotations) const {
	double output;

//	if(CommandBase::m_driveTrain->IsShifted() == false) {
//		output = rotations / m_encoderRevPerWheelRevHighGear;
//	}
//	else {
		output = rotations / m_encoderRevPerWheelRevLowGear;
//	}
	return output;
}

double GreyhillEncoder::ConvertWheelRotationsToEncoderRotations(double rotations) const {
	double output;

//	if(CommandBase::m_driveTrain->IsShifted() == false) {
//		output = rotations * m_encoderRevPerWheelRevHighGear;
//	}
//	else {
		output = rotations * m_encoderRevPerWheelRevLowGear;
//	}
	return output;
}

double GreyhillEncoder::ConvertWheelRotationsToDistance(double rotations) const {
	return rotations * m_inchesPerWheelRev;
}

double GreyhillEncoder::ConvertDistanceToWheelRotations(double distance) const {
	return distance / m_inchesPerWheelRev;
}

int GreyhillEncoder::ConvertEncoderRotationsToEncoderTicks(double rotations) const {
	return rotations * m_ticksPerRev;
}

double GreyhillEncoder::ConvertEncoderTicksToEncoderRotations(int ticks) const {
	return ticks / ((double) m_ticksPerRev);
}

void GreyhillEncoder::ResetDistance() {
	m_offset = GetRawDistance();
}
