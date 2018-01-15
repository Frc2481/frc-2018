#include <utils/Kinematics.h>
#include <cmath>
#include "RobotParameters.h"

RigidTransform2D::Delta Kinematics::forwardKinematics(double frDriveDelta, double flDriveDelta, double brDriveDelta, double blDriveDelta,
	double frRotationDelta, double flRotationDelta, double brRotationDelta, double blRotationDelta) 
{
	double linearVelocity = (frDriveDelta + flDriveDelta + brDriveDelta + blDriveDelta) / 4;
	double deltaV = (frDriveDelta - flDriveDelta - brDriveDelta - blDriveDelta) / 4;
	double deltaRotation = deltaV * 4 * Constants::kTrackScrubFactor / Constants::kTrackEffectiveDiameter;
	return RigidTransform2D::Delta(linearVelocity, 0, deltaRotation);
}

RigidTransform2D::Delta Kinematics::forwardKinematics(double frDriveDelta, double flDriveDelta, double brDriveDelta, double blDriveDelta,
	double frRotationDelta, double flRotationDelta, double brRotationDelta, double blRotationDelta, double gyroDelta)
{
	//TODO: Initialize these Values
	double L = ROBOT_LENGTH;
	double W = ROBOT_WIDTH;
	double originY, originX;
	originY = 0;
	originX = 0;

	double FR_B = sin(frRotationDelta) * frDriveDelta;
	double FR_D = cos(frRotationDelta) * frDriveDelta;
	
	double FL_B = sin(flRotationDelta) * flDriveDelta;
	double FL_C = cos(flRotationDelta) * flDriveDelta;

	double BR_A = sin(brRotationDelta) * brDriveDelta;
	double BR_D = cos(brRotationDelta) * brDriveDelta;

	double BL_A = sin(blRotationDelta) * blDriveDelta;
	double BL_C = cos(blRotationDelta) * blDriveDelta;

	double A = (BR_A + BL_A) / 2;
	double B = (FR_B + FL_B) / 2;
	double C = (FL_C + BL_C) / 2;
	double D = (FR_D + BR_D) / 2;

	double omega1, omega2, omega;
	omega1 = (B - A) / L;
	omega2 = (C - D) / W;
	omega = (omega1 + omega2) / 2;

	double STR, FWD, STR1, STR2, FWD1, FWD2;
	STR1 = omega * ((L / 2.0) + originY) + A;
	STR2 = -omega * ((L / 2.0) - originY) + B;
	FWD1 = omega * ((W / 2.0) + originX) + C;
	FWD2 = -omega * ((W / 2.0) - originX) + D;

	STR = (STR1 + STR2) / 2;
	FWD = (FWD1 + FWD2) / 2;

	return RigidTransform2D::Delta(FWD, STR, gyroDelta);
}

RigidTransform2D Kinematics::integrateForwardKinematics(RigidTransform2D currentPose, double frDriveDelta, double flDriveDelta, double brDriveDelta, double blDriveDelta,
	double frRotationDelta, double flRotationDelta, double brRotationDelta, double blRotationDelta, Rotation2D currentHeading) 
{
	RigidTransform2D::Delta withGyro = forwardKinematics(frDriveDelta, flDriveDelta, brDriveDelta, blDriveDelta,
		frRotationDelta, flRotationDelta, brRotationDelta, blRotationDelta, currentPose.getRotation().inverse().rotateBy(currentHeading).getRadians());

	return currentPose.transformBy(RigidTransform2D::fromVelocity(withGyro));
}

Kinematics::DriveVelocity::DriveVelocity(double fLeft, double fRight, double bLeft, double bRight) {
	m_fLeft = fLeft;
	m_fRight = fRight;
	m_bLeft = bLeft;
	m_bRight = bRight;
}

Kinematics::DriveVelocity Kinematics::DriveVelocity::inverseKinematics(RigidTransform2D::Delta velocity) {
	if (fabs(velocity.m_dtheta) < 1e-9) {
		return Kinematics::DriveVelocity(velocity.m_dx, velocity.m_dx, velocity.m_dx, velocity.m_dx);
	}
	double deltaV = Constants::kTrackEffectiveDiameter * velocity.m_dtheta / (2 * Constants::kTrackScrubFactor);
	return Kinematics::DriveVelocity(velocity.m_dx + deltaV, velocity.m_dx - deltaV, velocity.m_dx + deltaV, velocity.m_dx - deltaV);
}
