/*
 * InverseKinematicsV2.cpp
 *
 *  Created on: Aug 21, 2017
 *      Author: Team2481
 */

#include "InverseKinematicsV2.h"
#include <Components/SwerveModuleV2Constants.h>
#include <cmath>
#include <algorithm>


InverseKinematicsV2::InverseKinematicsV2() {
	// TODO Auto-generated constructor stub

}

InverseKinematicsV2::~InverseKinematicsV2() {
	// TODO Auto-generated destructor stub
}

void InverseKinematicsV2::SwerveInverseKinematics(Translation2D &translation,
		double rotation, double &wheelSpeedFR, double &wheelSpeedFL, double &wheelSpeedBR, double &wheelSpeedBL,
		Rotation2D &wheelAngleFL, Rotation2D &wheelAngleFR, Rotation2D &wheelAngleBL, Rotation2D &wheelAngleBR) {

	double A = translation.getX() - rotation * (SwerveModuleV2Constants::k_robotLength / 2.0);
	double B = translation.getX() + rotation * (SwerveModuleV2Constants::k_robotLength / 2.0);
	double C = translation.getY() - rotation * (SwerveModuleV2Constants::k_robotWidth / 2.0);
	double D = translation.getY() + rotation * (SwerveModuleV2Constants::k_robotWidth / 2.0);
	wheelSpeedFL = -sqrt(pow(B, 2) + pow(D, 2));
	wheelSpeedFR = sqrt(pow(B, 2) + pow(C, 2));
	wheelSpeedBR = sqrt(pow(A, 2) + pow(C, 2));
	wheelSpeedBL = -sqrt(pow(A, 2) + pow(D, 2));
	wheelAngleFL = Rotation2D(D,B,true);
	wheelAngleFR = Rotation2D(C,B,true);
	wheelAngleBL = Rotation2D(D,A,true);
	wheelAngleBR = Rotation2D(C,A,true);

	double maxWheelSpeed = std::max(wheelSpeedFL,std::max(wheelSpeedFR,std::max(wheelSpeedBL,wheelSpeedBR)));
	if (maxWheelSpeed > 1) {
		wheelSpeedFR /= maxWheelSpeed;
		wheelSpeedFL /= maxWheelSpeed;
		wheelSpeedBR /= maxWheelSpeed;
		wheelSpeedBL /= maxWheelSpeed;
	}
}

