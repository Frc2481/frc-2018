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
#include "WPILib.h"

class Kinematics {
public:
	Kinematics();
	virtual ~Kinematics();

	static void SwerveInverseKinematics(Translation2D &translation,
			double rotation, double &wheelSpeedFR, double &wheelSpeedFL, double &wheelSpeedBR, double &wheelSpeedBL,
			Rotation2D &wheelAngleFL, Rotation2D &wheelAngleFR, Rotation2D &wheelAngleBL, Rotation2D &wheelAngleBR);

	static RigidTransform2D SwerveForwardKinematics(Rotation2D flAngle, Translation2D flVelocity, Rotation2D frAngle,
			Translation2D frVelocity, Rotation2D blAngle, Translation2D blVelocity, Rotation2D brAngle, Translation2D brVelocity);
	RigidTransform2D IntegrateForwardKinematics(RigidTransform2D currentPose, RigidTransform2D::Delta forwardKinematics);
};

#endif /* KINEMATICS_H_ */
