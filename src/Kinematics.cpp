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

void Kinematics::SwerveInverseKinematics(Translation2D &velocity, double yawRate,
										double &wheelSpeedFR, double &wheelSpeedFL, double &wheelSpeedBR, double &wheelSpeedBL,
										Rotation2D &wheelAngleFL, Rotation2D &wheelAngleFR, Rotation2D &wheelAngleBL, Rotation2D &wheelAngleBR) {
	// +x = robot right
	// +y = robot forward
	// +yaw = CCW, zero is robot forward
	// +encoder yaw = CW, zero is robot forward

	// get robot dimensions
	double length = RobotParameters::k_robotLength;
	double width = RobotParameters::k_robotWidth;

	// calculate average velocities
	double VxBack = velocity.getX() + yawRate * (length / 2.0);
	double VxFront = velocity.getX() - yawRate * (length / 2.0);
	double VyRight = velocity.getY() + yawRate * (width / 2.0);
	double VyLeft = velocity.getY() - yawRate * (width / 2.0);

	// calculate wheel speed
	wheelSpeedFL = sqrt(pow(VxFront, 2) + pow(VyLeft, 2));
	wheelSpeedFR = sqrt(pow(VxFront, 2) + pow(VyRight, 2));
	wheelSpeedBR = sqrt(pow(VxBack, 2) + pow(VyRight, 2));
	wheelSpeedBL = sqrt(pow(VxBack, 2) + pow(VyLeft, 2));

	// calculate wheel angle
	wheelAngleFL = Rotation2D(VyLeft, VxFront, true);
	wheelAngleFR = Rotation2D(VyRight, VxFront, true);
	wheelAngleBL = Rotation2D(VyLeft, VxBack, true);
	wheelAngleBR = Rotation2D(VyRight, VxBack, true);

	// limit wheel speed
	double maxWheelSpeed = std::max(wheelSpeedFL, std::max(wheelSpeedFR, std::max(wheelSpeedBL, wheelSpeedBR)));
	if (maxWheelSpeed > 1) {
		wheelSpeedFR /= maxWheelSpeed;
		wheelSpeedFL /= maxWheelSpeed;
		wheelSpeedBR /= maxWheelSpeed;
		wheelSpeedBL /= maxWheelSpeed;
	}
}

RigidTransform2D::Delta Kinematics::SwerveForwardKinematics(Rotation2D flAngle, RigidTransform2D::Delta flVelocity, Rotation2D frAngle,
		RigidTransform2D::Delta frVelocity, Rotation2D blAngle, RigidTransform2D::Delta blVelocity, Rotation2D brAngle, RigidTransform2D::Delta brVelocity) {
	// +x = robot right
	// +y = robot forward
	// +yaw = CCW, zero is robot forward
	// +encoder yaw = CW, zero is robot forward

	// get robot dimensions
	double length = RobotParameters::k_robotLength;
	double width = RobotParameters::k_robotWidth;

	// get x and y components of wheel velocities
	double VyFR = frVelocity.GetX() * frAngle.getCos();
	double VxFR = frVelocity.GetX() * frAngle.getSin();

	double VyBR = brVelocity.GetX() * brAngle.getCos();
	double VxBR = brVelocity.GetX() * brAngle.getSin();

	double VyBL = blVelocity.GetX() * blAngle.getCos();
	double VxBL = blVelocity.GetX() * blAngle.getSin();

	double VyFL = flVelocity.GetX() * flAngle.getCos();
	double VxFL = flVelocity.GetX() * flAngle.getSin();

	// calculate average velocities
	double VxBack = (VxBR + VxBL) / 2.0;
	double VxFront = (VxFR + VxFL) / 2.0;
	double VyRight = (VyFR + VyBR) / 2.0;
	double VyLeft = (VyFL + VyBL) / 2.0;

	// calculate robot yaw rate
	double yawRate1 = (VxBack - VxFront) / length;
	double yawRate2 = (VyRight - VyLeft) / width;
	double yawRate = (yawRate1 + yawRate2) / 2.0;

	// calculate robot center velocity
	double Vxc1 = VxBack - yawRate * length / 2.0;
	double Vxc2 = VxFront + yawRate * length / 2.0;
	double Vyc1 = VyRight - yawRate * width / 2.0;
	double Vyc2 = VyLeft + yawRate * width / 2.0;
	double Vxc = (Vxc1 + Vxc2) / 2.0;
	double Vyc = (Vyc1 + Vyc2) / 2.0;

	return RigidTransform2D::Delta::fromDelta(Vxc, Vyc, yawRate, flVelocity.GetDt());
}
