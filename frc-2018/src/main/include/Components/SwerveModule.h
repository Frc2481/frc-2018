/*
 * SwerveModule.h
 *
 *  Created on: Jul 17, 2017
 *      Author: Team2481
 */

#ifndef SRC_SWERVEMODULE_H_
#define SRC_SWERVEMODULE_H_

#include "ctre/Phoenix.h"
#include "utils\Translation2D.h"

class CTREMagEncoder;
class GreyhillEncoder;

class SwerveModule {
private:
	TalonSRX *m_steerMotor;
	TalonSRX *m_driveMotor;
	CTREMagEncoder *m_steerEncoder;
	GreyhillEncoder *m_driveEncoder;
	bool m_optimizationEnabled;
	bool m_angleOptimized;
	bool m_isMoving;
	bool m_isCloseLoopControl;
	bool m_motionMagic;
	std::string m_name;

public:
	SwerveModule(uint32_t driveID, uint32_t steerID, const std::string name);
	virtual ~SwerveModule();

	Rotation2D GetAngle() const;
	void SetOptimized(bool isOptimized);
	bool GetOptimized();
	void SetAngle(Rotation2D angle, bool force = false);
	bool IsSteerOnTarget() const;

	void SetOpenLoopSpeed(double speed);
	void SetOpenLoopSteer(double speed);
	double GetSpeed() const;
	void SetCloseLoopDriveDistance(Translation2D distance);
	void DisableCloseLoopDrive();
	Translation2D GetDistance() const;
	void ZeroDriveDistance();
	double GetDistanceError() const;
	bool IsDriveOnTarget() const;

	void Set(double speed, Rotation2D angle);

	void SetBrake(bool brake);

	void SetMagicAccel(double accel);

	CTREMagEncoder* GetSteerEncoder();
	GreyhillEncoder* GetDriveEncoder();
	double GetSteerCurrent() const;
	double GetDriveCurrent() const;
	void Periodic();
};

#endif /* SRC_SWERVEMODULE_H_ */
