/*
 * Arm.h
 *
 *  Created on: Jan 16, 2018
 *      Author: Team2481
 */

#ifndef SRC_SUBSYSTEMS_ARM_H_
#define SRC_SUBSYSTEMS_ARM_H_

#include "ctre/Phoenix.h"
#include "WPILib.h"
#include "utils/Rotation2D.h"
#include "utils/Translation2D.h"
#include "utils/RigidTransform2D.h"
#include "../Components/CTREMagEncoder.h"

class Arm {
private:
	TalonSRX* m_extensionMotor;
	TalonSRX* m_pivotMotor;
	CTREMagEncoder* m_extensionEncoder;
	CTREMagEncoder* m_pivotEncoder;

	Translation2D k_baseArmLength;
	Translation2D k_pivotHight;

public:
	Arm();
	virtual ~Arm();

	Rotation2D GetPivotAngle();
	Translation2D GetExtensionPosition();
	RigidTransform2D GetEndEfector();

};

#endif /* SRC_SUBSYSTEMS_ARM_H_ */
