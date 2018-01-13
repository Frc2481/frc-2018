/*
 * DriveTrain2017.h
 *
 *  Created on: Aug 28, 2017
 *      Author: Team2481
 */

#ifndef SRC_SUBSYSTEMS_DRIVETRAIN2017_H_
#define SRC_SUBSYSTEMS_DRIVETRAIN2017_H_

#include "Commands/Subsystem.h"
#include "Solenoid.h"
#include "utils/Rotation2D.h"
#include "utils/Translation2D.h"
#include "Observer.h"
#include "Kinematics.h"

class SwerveModuleV2;
class AHRS;

class DriveTrain2017 : public Subsystem{
private:
	SwerveModuleV2 *m_flWheel;
	SwerveModuleV2 *m_frWheel;
	SwerveModuleV2 *m_brWheel;
	SwerveModuleV2 *m_blWheel;
	Solenoid *m_shifter;

	AHRS* m_imu;
	bool m_isFieldCentric;
	bool m_isForward;
	double m_xPos, m_yPos, m_twist;
	float m_prevAngle;
	float m_pHeadingCorrection;
	float m_originX;
	float m_originY;

	double m_encRotationPerDegrees;
	Rotation2D m_headingCorrectionOffset;

	float m_heading;
	bool m_headingCorrection;
	float m_roll;
	float m_pitch;


	Translation2D m_motionSetpoint;

	Observer m_observer;

public:
	enum SwerveModuleType{
		FRONT_LEFT_MODULE,
		FRONT_RIGHT_MODULE,
		BACK_LEFT_MODULE,
		BACK_RIGHT_MODULE,
	};
	DriveTrain2017();
	virtual ~DriveTrain2017();
	void InitDefaultCommand();
	void Drive(double xPos, double yPos, double twist);
	void SetOrigin(double xPos, double yPos);
	double GetXOrigin() const;
	double GetYOrigin() const;
	float GetRoll() const;
	float GetPitch() const;
	void Stop();
	void SetFieldCentric(bool fieldCentric);
	void SetForward(bool forward);
	void SetHeadingCorrection(bool headingCorrection);
	const Rotation2D& GetGyroCorrectionOffset() const;
	void SetGyroCorrectionOffset(Rotation2D &offset);
	void ZeroGyro();
	bool IsHeadingCorrection() const;
	void PeriodicUpdate();
	void SetBrake(bool brake);
	void Shift(bool state);
	bool IsShifted() const;
	class SwerveModuleV2* GetModule(DriveTrain2017::SwerveModuleType module) const;
	Rotation2D GetHeading() const;
	void DriveCloseLoopDistance(Translation2D setpoint);

	Translation2D GetMotionMagicSetpoint() const;

	double ComputeDriveDistanceInchestoEncoderRotations(double inches);
	double ComputeDegreesToEncoderRotations(double degrees);

	void SetMotionMagicAccel(double accel);
	double GetDriveDistance() const;

	bool IsSteerOnTarget() const;
	bool IsDriveOnTarget() const;

	virtual void Periodic();

};

#endif /* SRC_SUBSYSTEMS_DRIVETRAIN2017_H_ */
