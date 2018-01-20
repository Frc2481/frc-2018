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

RigidTransform2D Kinematics::SwerveForwardKinematics(Rotation2D flAngle, Translation2D flVelocity, Rotation2D frAngle,
		Translation2D frVelocity, Rotation2D blAngle, Translation2D blVelocity, Rotation2D brAngle, Translation2D brVelocity) {
	double length = SwerveModuleV2Constants::k_robotLength;
	double width = SwerveModuleV2Constants::k_robotWidth;
	double originY, originX;
//	originY = 0.0;
//	originX = 0.0;

	double rx1 = width / 2.0;
	double ry1 = length / 2.0;

	double rx3 = -width / 2.0;
	double ry3 = -length / 2.0;

	double Vxw1 = frVelocity.getX() * frAngle.getCos();
	double Vyw1 = frVelocity.getX() * frAngle.getSin();

	double Vxw2 = brVelocity.getX() * brAngle.getCos();
	double Vyw2 = brVelocity.getX() * brAngle.getSin();

	double Vxw3 = blVelocity.getX() * blAngle.getCos();
	double Vyw3 = blVelocity.getX() * blAngle.getSin();

	double Vxw4 = flVelocity.getX() * flAngle.getCos();
	double Vyw4 = flVelocity.getX() * flAngle.getSin();

	double A = (Vyw2 + Vyw3) / 2.0; // average back y vel
	double B = (Vyw1 + Vyw4) / 2.0; // average front y vel
	double C = (Vxw4 + Vxw3) / 2.0; // average left x vel
	double D = (Vxw1 + Vxw2) / 2.0; // average right x vel

	double omega1 = (B - A) / length; // this is length
	double omega2 = (C - D) / width; // this is also length
	double omega = (omega1 + omega2) / 2.0;

	SmartDashboard::PutNumber("omega 1", omega1);
	SmartDashboard::PutNumber("omega 2", omega2);

	double Vxc1 = Vxw1 + omega * ry1;
	double Vxc2 = Vxw3 + omega * ry3;

	double Vyc1 = Vyw1 - omega * rx1;
	double Vyc2 = Vyw3 - omega * rx3;

	double Vxc = (Vxc1 + Vxc2) / 2.0;
	double Vyc = (Vyc1 + Vyc2) / 2.0;

	Translation2D translate(Vxc, Vyc);
	Rotation2D rotate = Rotation2D::fromRadians(omega);
	RigidTransform2D transform(translate, rotate);

	return transform;
}

RigidTransform2D Kinematics::IntegrateForwardKinematics(RigidTransform2D currentPose, RigidTransform2D::Delta forwardKinematics) {
	return currentPose.transformBy(RigidTransform2D::fromVelocity(forwardKinematics));
}
