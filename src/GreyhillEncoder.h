/*
 * GreyhillEncoder.h
 *
 *  Created on: Jul 10, 2017
 *      Author: Team2481
 */

#ifndef SRC_GREYHILLENCODER_H_
#define SRC_GREYHILLENCODER_H_

#include "ctre/Phoenix.h"
#include "utils/Translation2D.h"
#include "Subsystems/DriveTrain2017.h"

class GreyhillEncoder {
	private:
	TalonSRX* m_talon;
	std::string m_name;
	std::string m_calibrationKey;
	Translation2D m_offset;
	int m_ticksPerRev;
	double m_inchesPerWheelRev;
	double m_encoderRevPerWheelRev;

public:
	GreyhillEncoder(TalonSRX* talon, const std::string& name, int ticksPerEncoderRev, double inchesPerWheelRev, double encoderRevPerWheelRev);
	virtual ~GreyhillEncoder();
	Translation2D GetRawDistance() const;
	Translation2D GetDistance() const;
	double GetEncoderSpeed() const;
	int GetEncoderTicks() const;
	double ConvertEncoderRotationsToWheelRotations(double rotations) const;
	double ConvertWheelRotationsToEncoderRotations(double rotations) const;
	double ConvertWheelRotationsToDistance(double rotations) const;
	double ConvertDistanceToWheelRotations(double distance) const;
	int ConvertEncoderRotationsToEncoderTicks(double rotations) const;
	double ConvertEncoderTicksToEncoderRotations(int ticks) const;
	void SetEncoderTicks(int ticks, int timeOut = 0);
	void ResetDistance();
};

#endif /* SRC_GREYHILLENCODER_H_ */
