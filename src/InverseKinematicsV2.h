/*
 * InverseKinematicsV2.h
 *
 *  Created on: Aug 21, 2017
 *      Author: Team2481
 */

#ifndef INVERSEKINEMATICSV2_H_
#define INVERSEKINEMATICSV2_H_

#include "utils/Translation2D.h"

class InverseKinematicsV2 {
public:
	InverseKinematicsV2();
	virtual ~InverseKinematicsV2();

	static void SwerveInverseKinematics(Translation2D &translation,
			double rotation, double &wheelSpeedFR, double &wheelSpeedFL, double &wheelSpeedBR, double &wheelSpeedBL,
			Rotation2D &wheelAngleFL, Rotation2D &wheelAngleFR, Rotation2D &wheelAngleBL, Rotation2D &wheelAngleBR);

};

#endif /* INVERSEKINEMATICSV2_H_ */
