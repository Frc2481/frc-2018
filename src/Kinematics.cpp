/*
 * Kinematics.cpp
 *
 *  Created on: Aug 21, 2017
 *      Author: Team2481
 */

#include "Kinematics.h"
#include <RobotParameters.h>
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

	double A = translation.getX() - rotation * (RobotParameters::k_robotLength / 2.0);
	double B = translation.getX() + rotation * (RobotParameters::k_robotLength / 2.0);
	double C = translation.getY() - rotation * (RobotParameters::k_robotWidth / 2.0);
	double D = translation.getY() + rotation * (RobotParameters::k_robotWidth / 2.0);
	wheelSpeedFL = sqrt(pow(B, 2) + pow(D, 2));
	wheelSpeedFR = sqrt(pow(B, 2) + pow(C, 2));
	wheelSpeedBR = sqrt(pow(A, 2) + pow(C, 2));
	wheelSpeedBL = sqrt(pow(A, 2) + pow(D, 2));
	wheelAngleFL = Rotation2D(D, B, true);
	wheelAngleFR = Rotation2D(C, B, true);
	wheelAngleBL = Rotation2D(D, A, true);
	wheelAngleBR = Rotation2D(C, A, true);

	double maxWheelSpeed = std::max(wheelSpeedFL,std::max(wheelSpeedFR,std::max(wheelSpeedBL,wheelSpeedBR)));
	if (maxWheelSpeed > 1) {
		wheelSpeedFR /= maxWheelSpeed;
		wheelSpeedFL /= maxWheelSpeed;
		wheelSpeedBR /= maxWheelSpeed;
		wheelSpeedBL /= maxWheelSpeed;
	}
}

RigidTransform2D::Delta Kinematics::SwerveForwardKinematics(Rotation2D flAngle, RigidTransform2D::Delta flVelocity, Rotation2D frAngle,
		RigidTransform2D::Delta frVelocity, Rotation2D blAngle, RigidTransform2D::Delta blVelocity, Rotation2D brAngle, RigidTransform2D::Delta brVelocity) {
	double length = RobotParameters::k_robotLength;
	double width = RobotParameters::k_robotWidth;

	double rx = width / 2.0;
	double ry = length / 2.0;

	double Vxw1 = frVelocity.GetX() * frAngle.getCos();
	double Vyw1 = frVelocity.GetX() * frAngle.getSin();

	double Vxw2 = brVelocity.GetX() * brAngle.getCos();
	double Vyw2 = brVelocity.GetX() * brAngle.getSin();

	double Vxw3 = blVelocity.GetX() * blAngle.getCos();
	double Vyw3 = blVelocity.GetX() * blAngle.getSin();

	double Vxw4 = flVelocity.GetX() * flAngle.getCos();
	double Vyw4 = flVelocity.GetX() * flAngle.getSin();

	double A = (Vyw2 + Vyw3) / 2.0; // average back y vel
	double B = (Vyw1 + Vyw4) / 2.0; // average front y vel
	double C = (Vxw1 + Vxw2) / 2.0; // average right x vel
	double D = (Vxw4 + Vxw3) / 2.0; // average left x vel

	double omega1 = (B - A) / length;
	double omega2 = (D - C) / width;
	double omega = (omega1 + omega2) / 2.0;

	double Vxc1 = omega * ry + A;
	double Vxc2 = -omega * ry + B;
	double Vyc1 = omega * rx + C;
	double Vyc2 = -omega * rx + D;

	double Vxc = (Vxc1 + Vxc2) / 2.0;
	double Vyc = (Vyc1 + Vyc2) / 2.0;

	return RigidTransform2D::Delta::fromDelta(Vxc, Vyc, omega, flVelocity.GetDt());
}

RigidTransform2D Kinematics::IntegrateForwardKinematics(RigidTransform2D currentPose, RigidTransform2D::Delta forwardKinematics) {
	return currentPose.transformBy(RigidTransform2D::fromVelocity(forwardKinematics));
}
