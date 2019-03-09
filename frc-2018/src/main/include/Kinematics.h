/*
 * Kinematics.h
 *
 *  Created on: Aug 21, 2017
 *      Author: Team2481
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include "utils/Translation2D.h"
#include "utils/RigidTransform2D.h"

class Kinematics {
public:
	Kinematics();
	virtual ~Kinematics();

	static void SwerveInverseKinematics(Translation2D &velocity, double yawRate,
			double &wheelSpeedFL, double &wheelSpeedFR, double &wheelSpeedBL, double &wheelSpeedBR,
			Rotation2D &wheelAngleFL, Rotation2D &wheelAngleFR, Rotation2D &wheelAngleBL, Rotation2D &wheelAngleBR);

	static RigidTransform2D::Delta SwerveForwardKinematics(Rotation2D& flAngle, RigidTransform2D::Delta& flVelocity, Rotation2D& frAngle,
			RigidTransform2D::Delta& frVelocity, Rotation2D& blAngle, RigidTransform2D::Delta& blVelocity, Rotation2D& brAngle, RigidTransform2D::Delta& brVelocity);
};

#endif /* KINEMATICS_H_ */
