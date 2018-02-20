/*
 * Arm.h
 *
 *  Created on: Jan 25, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_SUBSYSTEMS_ARM_H_
#define SRC_SUBSYSTEMS_ARM_H_

#include "ctre/Phoenix.h"
#include "WPILib.h"
#include "RobotMap.h"
#include "RobotParameters.h"
#include "Commands/Subsystem.h"
#include "ArmExtension.h"

class Arm : public Subsystem{
private:
	TalonSRX* m_extenderMaster;
	TalonSRX* m_extenderSlave;
	TalonSRX* m_pivot;
	bool m_prevExtensionTravellingDown;
	double m_desiredExtensionSetpoint;
	ArmExtension m_armConstraints;
	Rotation2D m_pivotAngle;
	double m_scale;
	bool m_isPivotZeroed;
	bool m_isExtensionZeroed;

public:
	Arm();
	virtual ~Arm();
	void SetExtensionPosition(double position); //raw control of extension-> be careful of constraints
	void SetDesiredExtension(double extension); //safely control extension obeying constraints
	double GetDesiredExtension();
	double GetExtensionPosition();
	bool IsExtensionOnTarget();
	void ZeroExtension();
	void SetExtensionOpenLoop(double speed);
	void SetPivotOpenLoop(double speed);
	void SetPivotAngle(Rotation2D angle);
	bool IsPivotOnTarget();
	Rotation2D GetPivotAngle();
	Rotation2D GetDesiredPivotAngle();
	void ZeroPivot();
	void SetPivotAccel(int accel);
	virtual void Periodic();
	double GetAllowedExtensionPos();
	double ConvertInchesToEncTicks(double inches);
	double ConvertEncTicksToInches(double ticks);
	double GetLastCommandedSetpoint();
	double SetExtentionMotionScaling(double scale);
	void SetIntakePos(int intakePos);
	int GetIntakePos();

};

#endif /* SRC_SUBSYSTEMS_ARM_H_ */
