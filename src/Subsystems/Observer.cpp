/*
 * Observer.cpp
 *
 *  Created on: Jan 9, 2018
 *      Author: FIRSTMentor
 */

#include <Subsystems/Observer.h>
#include "SwerveModuleV2Constants.h"
#include <cmath>
#include <algorithm>

Observer::Observer() {

}

Observer::~Observer() {
	// TODO Auto-generated destructor stub
}

RigidTransform2D Observer::ForwardKinematicsDriveTrain(Rotation2D flAngle, Translation2D flVelocity, Rotation2D frAngle,
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

void Observer::AddGyroObservation(Rotation2D gyroAngleVel, double timeStamp) {
	m_gyroAngleVel.put(InterpolatingDouble(timeStamp), gyroAngleVel);
}

void Observer::AddDriveTrainObservation(RigidTransform2D robotCenterVel,
		double timeStamp) {
	m_driveTrainVel.put(InterpolatingDouble(timeStamp), centerVel);
}

void Observer::TimeUpdateRobotState(timestamp) {
	center
}
