/*
 * CTREMagEncoder.h
 *
 *  Created on: Jul 3, 2017
 *      Author: Team2481
 */

#ifndef SRC_CTREMAGENCODER_H_
#define SRC_CTREMAGENCODER_H_
#include "utils/Rotation2D.h"
#include "ctre/Phoenix.h"
#include <string>

using namespace ctre::phoenix::motorcontrol::can;

class CTREMagEncoder {
private:
	TalonSRX* m_talon;
	std::string m_name;
	std::string m_calibrationKey;
	Rotation2D m_offset;
	Rotation2D m_cachedAngle;
	int m_cachedTicks;
public:
	void Periodic();
	CTREMagEncoder(TalonSRX *talon, std::string name);
	virtual ~CTREMagEncoder();
	Rotation2D GetRawAngle() const;
	const Rotation2D& GetAngle(bool cached=true);
	int GetRotations() const;
	int GetEncoderTicks(bool overflow = false, bool cached = true) const;
	void Calibrate();
	int ConvertAngleToSetpoint(Rotation2D targetAngle);
	int ConvertAngleToEncoderTicks(Rotation2D angle);
	void SetEncoderRaw(int ticks);
	bool IsConnected();
	bool IsCalibrated();

};

#endif /* SRC_CTREMAGENCODER_H_ */
