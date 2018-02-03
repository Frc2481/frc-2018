/*
 * Arm.cpp
 *
 *  Created on: Jan 16, 2018
 *      Author: Team2481
 */

#include <Subsystems/Arm.h>

Arm::Arm() {
	// TODO Auto-generated constructor stub

}

Arm::~Arm() {
	// TODO Auto-generated destructor stub
}

Rotation2D Arm::GetPivotAngle() {
	return m_pivotEncoder->GetAngle();
}

Translation2D Arm::GetExtensionPosition() {

}

RigidTransform2D Arm::GetEndEfector() {

}
