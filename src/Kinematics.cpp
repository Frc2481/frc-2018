/*
 * Kinematics.cpp
 *
 *  Created on: Aug 21, 2017
 *      Author: Team2481
 */

#include "Kinematics.h"
#include <Components/SwerveModuleV2Constants.h>
#include <algorithm>
#include <cmath>


Kinematics::Kinematics() {
	// TODO Auto-generated constructor stub

}

Kinematics::~Kinematics() {
	// TODO Auto-generated destructor stub
}

void Kinematics::SwerveInverseKinematics(Translation2D &translation,
		double rotation, double &wheelSpeedFR, double &wheelSpeedFL, double &wheelSpeedBR, double &wheelSpeedBL,
		Rotation2D &wheelAngleFL, Rotation2D &wheelAngleFR, Rotation2D &wheelAngleBL, Rotation2D &wheelAngleBR) {

	double A = translation.getX() - rotation * (SwerveModuleV2Constants::k_robotLength / 2.0);
	double B = translation.getX() + rotation * (SwerveModuleV2Constants::k_robotLength / 2.0);
	double C = translation.getY() - rotation * (SwerveModuleV2Constants::k_robotWidth / 2.0);
	double D = translation.getY() + rotation * (SwerveModuleV2Constants::k_robotWidth / 2.0);
	wheelSpeedFL = sqrt(pow(B, 2) + pow(D, 2));
	wheelSpeedFR = sqrt(pow(B, 2) + pow(C, 2));
	wheelSpeedBR = sqrt(pow(A, 2) + pow(D, 2));
	wheelSpeedBL = sqrt(pow(A, 2) + pow(C, 2));
	wheelAngleFL = Rotation2D(B,D,true);
	wheelAngleFR = Rotation2D(B,C,true);
	wheelAngleBL = Rotation2D(A,D,true);
	wheelAngleBR = Rotation2D(A,C,true);

	double maxWheelSpeed = std::max(wheelSpeedFL,std::max(wheelSpeedFR,std::max(wheelSpeedBL,wheelSpeedBR)));
	if (maxWheelSpeed > 1) {
		wheelSpeedFR /= maxWheelSpeed;
		wheelSpeedFL /= maxWheelSpeed;
		wheelSpeedBR /= maxWheelSpeed;
		wheelSpeedBL /= maxWheelSpeed;
	}
}

RigidTransform2D Kinematics::ForwardKinematicsDriveTrain(Rotation2D flAngle, Translation2D flVelocity, Rotation2D frAngle,
										  Translation2D frVelocity, Rotation2D blAngle, Translation2D blVelocity,
										  Rotation2D brAngle, Translation2D brVelocity) {
	double length = SwerveModuleV2Constants::k_robotLength;
	double width = SwerveModuleV2Constants::k_robotWidth;
	double originY, originX;
	originY = 0;
	originX = 0;

	double FR_B = frAngle.getSin() * frVelocity;
	double FR_D = frAngle.getCos() * frVelocity;

	double FL_B = flAngle.getSin() * flVelocity;
	double FL_C = flAngle.getCos() * flVelocity;

	double BR_A = brAngle.getSin() * brVelocity;
	double BR_D = brAngle.getCos() * brVelocity;

	double BL_A = blAngle.getSin() * blVelocity;
	double BL_C = blAngle.getCos() * blVelocity;

	double A = (BR_A + BL_A) / 2;
	double B = (FR_B + FL_B) / 2;
	double C = (FL_C + BL_C) / 2;
	double D = (FR_D + BR_D) / 2;

	double omega1, omega2, omega;
	omega1 = (B - A) / length;
	omega2 = (C - D) / width;
	omega = (omega1 + omega2) / 2;

	double Vyc, Vxc, Vyc1, Vyc2, Vxc1, Vxc2;
	double rx = width / 2.0;
	double ry = length / 2.0;
	Vyc1 = omega * (ry + originY) + A;
	Vyc2 = -omega * (ry - originY) + B;
	Vxc1 = omega * (rx + originX) + C;
	Vxc2 = -omega * (rx - originX) + D;

	Vyc = (Vyc1 + Vyc2) / 2;
	Vxc = (Vxc1 + Vxc2) / 2;

	return RigidTransform2D(Vxc, Vyc, omega);
}

