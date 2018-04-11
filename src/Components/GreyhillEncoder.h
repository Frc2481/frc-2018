/*
 * GreyhillEncoder.h
 *
 *  Created on: Jul 10, 2017
 *      Author: Team2481
 */

#ifndef SRC_GREYHILLENCODER_H_
#define SRC_GREYHILLENCODER_H_

#include <Subsystems/DriveTrain.h>
#include "ctre/Phoenix.h"
#include "utils/Translation2D.h"
#include "CommandBase.h"

class GreyhillEncoder {
	private:
	TalonSRX* m_talon;
	std::string m_name;
	std::string m_calibrationKey;
	Translation2D m_offset;
	int m_ticksPerRev;
	double m_inchesPerWheelRev;
	double m_encoderRevPerWheelRevLowGear;
	double m_encoderRevPerWheelRevHighGear;
	int m_cachedTicks;

public:
	GreyhillEncoder(TalonSRX* talon, std::string name, int ticksPerEncoderRev, double inchesPerWheelRev,
				    double encoderRevPerWheelRevLowGear, double encoderRevPerWheelRevHighGear);
	virtual ~GreyhillEncoder();
	Translation2D GetRawDistance() const;
	Translation2D GetDistance() const;
	double GetEncoderSpeed() const;
	int GetEncoderTicks(bool cached = true) const;
	double ConvertEncoderRotationsToWheelRotations(double rotations) const;
	double ConvertWheelRotationsToEncoderRotations(double rotations) const;
	double ConvertWheelRotationsToDistance(double rotations) const;
	double ConvertDistanceToWheelRotations(double distance) const;
	int ConvertEncoderRotationsToEncoderTicks(double rotations) const;
	double ConvertEncoderTicksToEncoderRotations(int ticks) const;
	void SetEncoderTicks(int ticks, int timeOut = 0);
	void ResetDistance();
	void Periodic();
};

#endif /* SRC_GREYHILLENCODER_H_ */
