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
#include <WPILib.h>


Kinematics::Kinematics() {
	// TODO Auto-generated constructor stub

}

Kinematics::~Kinematics() {
	// TODO Auto-generated destructor stub
}

void Kinematics::SwerveInverseKinematics(Translation2D &velocity, double yawRate,
										double &wheelSpeedFL, double &wheelSpeedFR, double &wheelSpeedBL, double &wheelSpeedBR,
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

Translation2D Kinematics::CalculateWheelSlip(Rotation2D& wheelAngle, RigidTransform2D::Delta& wheelVelocity, Translation2D& robotAccel)
{
	double vy_wheel = wheelVelocity.GetX();
	Translation2D accelWheelFrame = robotAccel.rotateBy(wheelAngle);
	double alpha_slip = Preferences::GetInstance()->GetDouble("CORNERING_COEFF") * robotAccel.getX() * copysign(1.0, vy_wheel);
	double vx_wheel = vy_wheel * -alpha_slip;
	Translation2D newWheelVel = Translation2D(vx_wheel, vy_wheel);
	return newWheelVel.rotateBy(wheelAngle.inverse());
}

RigidTransform2D::Delta Kinematics::SwerveForwardKinematics(Rotation2D& flAngle, RigidTransform2D::Delta& flVelocity, Rotation2D& frAngle,
		RigidTransform2D::Delta& frVelocity, Rotation2D& blAngle, RigidTransform2D::Delta& blVelocity, Rotation2D& brAngle, RigidTransform2D::Delta& brVelocity, Rotation2D& yawRate, Translation2D& robotAccel) {
	// +x = robot right
	// +y = robot forward
	// +yaw = CCW, zero is robot forward
	// +encoder yaw = CW, zero is robot forward

	// get robot dimensions
	double length = RobotParameters::k_robotLength;
	double width = RobotParameters::k_robotWidth;

	int useFL = fabs(flVelocity.GetX()) > 0.01;
	int useFR = fabs(frVelocity.GetX()) > 0.01;
	int useBL = fabs(blVelocity.GetX()) > 0.01;
	int useBR = fabs(brVelocity.GetX()) > 0.01;


	Translation2D newFLVel = CalculateWheelSlip(flAngle, flVelocity, robotAccel);
	Translation2D newFRVel = CalculateWheelSlip(frAngle, frVelocity, robotAccel);
	Translation2D newBLVel = CalculateWheelSlip(blAngle, blVelocity, robotAccel);
	Translation2D newBRVel = CalculateWheelSlip(brAngle, brVelocity, robotAccel);

	// get x and y components of wheel velocities
	double VyFR = newFRVel.getY() * RobotParameters::k_frRadiusPercent; //frVelocity.GetX() * frAngle.getCos() * RobotParameters::k_frRadiusPercent;
	double VxFR = newFRVel.getX() * RobotParameters::k_frRadiusPercent; //frVelocity.GetX() * frAngle.getSin() * RobotParameters::k_frRadiusPercent;

	double VyBR = newBRVel.getY() * RobotParameters::k_brRadiusPercent; //brVelocity.GetX() * brAngle.getCos() * RobotParameters::k_brRadiusPercent;
	double VxBR = newBRVel.getX() * RobotParameters::k_brRadiusPercent; //brVelocity.GetX() * brAngle.getSin() * RobotParameters::k_brRadiusPercent;

	double VyBL = newBLVel.getY() * RobotParameters::k_blRadiusPercent; //blVelocity.GetX() * blAngle.getCos() * RobotParameters::k_blRadiusPercent;
	double VxBL = newBLVel.getX() * RobotParameters::k_blRadiusPercent; //blVelocity.GetX() * blAngle.getSin() * RobotParameters::k_blRadiusPercent;

	double VyFL = newFLVel.getY() * RobotParameters::k_flRadiusPercent; //flVelocity.GetX() * flAngle.getCos() * RobotParameters::k_flRadiusPercent;
	double VxFL = newFLVel.getX() * RobotParameters::k_flRadiusPercent; //flVelocity.GetX() * flAngle.getSin() * RobotParameters::k_flRadiusPercent;

//	// calculate average velocities
//	double VxBack = (VxBR + VxBL) / 2.0;
//	double VxFront = (VxFR + VxFL) / 2.0;
//	double VyRight = (VyFR + VyBR) / 2.0;
//	double VyLeft = (VyFL + VyBL) / 2.0;
//
//	// calculate robot yaw rate
//	double yawRate1 = (VxBack - VxFront) / length;
//	double yawRate2 = (VyRight - VyLeft) / width;
//	double yawRate = (yawRate1 + yawRate2) / 2.0;
//
//	// calculate robot center velocity
//	double Vxc1 = VxBack - yawRate * length / 2.0;
//	double Vxc2 = VxFront + yawRate * length / 2.0;
//	double Vyc1 = VyRight - yawRate * width / 2.0;
//	double Vyc2 = VyLeft + yawRate * width / 2.0;
//	double Vxc = (Vxc1 + Vxc2) / 2.0;
//	double Vyc = (Vyc1 + Vyc2) / 2.0;

	double Vxcfl = VxFL - yawRate.getRadians() * length / 2.0;
	double Vycfl = VyFL + yawRate.getRadians() * -width / 2.0;
	double Vxcfr = VxFR - yawRate.getRadians() * length / 2.0;
	double Vycfr = VyFR + yawRate.getRadians() * width / 2.0;
	double Vxcbl = VxBL - yawRate.getRadians() * -length / 2.0;
	double Vycbl = VyBL + yawRate.getRadians() * -width / 2.0;
	double Vxcbr = VxBR - yawRate.getRadians() * -length / 2.0;
	double Vycbr = VyBR + yawRate.getRadians() * width / 2.0;

	int useSum = useFL + useFR + useBL + useBR;

	double Vxc = 0;
	double Vyc = 0;

	if(useSum > 0) {
		Vxc = ((Vxcfl * useFL) + (Vxcfr * useFR) + (Vxcbl * useBL) + (Vxcbr * useBR)) / (useSum);
		Vyc = ((Vycfl * useFL) + (Vycfr * useFR) + (Vycbl * useBL) + (Vycbr * useBR)) / (useSum);
	}

	return RigidTransform2D::Delta::fromDelta(Vxc, Vyc, yawRate.getRadians(), flVelocity.GetDt());
}
