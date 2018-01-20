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
#include "Subsystems/Observer.h"
#include "Kinematics.h"
#include "SwerveModuleV2.h"


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

	Rotation2D m_oldFlAngle;
	Rotation2D m_oldFrAngle;
	Rotation2D m_oldBlAngle;
	Rotation2D m_oldBrAngle;

	Translation2D m_oldFlDistance;
	Translation2D m_oldFrDistance;
	Translation2D m_oldBlDistance;
	Translation2D m_oldBrDistance;

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

	Rotation2D GetOldFlAngle();
	Rotation2D GetOldFrAngle();
	Rotation2D GetOldBlAngle();
	Rotation2D GetOldBrAngle();

	void SetOldFlAngle(Rotation2D angle);
	void SetOldFrAngle(Rotation2D angle);
	void SetOldBlAngle(Rotation2D angle);
	void SetOldBrAngle(Rotation2D angle);

	Translation2D GetOldFlDistance();
	Translation2D GetOldFrDistance();
	Translation2D GetOldBlDistance();
	Translation2D GetOldBrDistance();

	void SetOldFlDistance(Translation2D distance);
	void SetOldFrDistance(Translation2D distance);
	void SetOldBlDistance(Translation2D distance);
	void SetOldBrDistance(Translation2D distance);

	virtual void Periodic();


	void CheckDiagnostics();


	double m_timestamp;
};

#endif /* SRC_SUBSYSTEMS_DRIVETRAIN2017_H_ */
