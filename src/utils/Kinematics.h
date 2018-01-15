#pragma once
#include <utils/Constants.h>
#include <utils/RigidTransform2D.h>
#include <utils/Rotation2D.h>

class Kinematics {
public:
	class DriveVelocity {
	public:
		double m_fLeft;
		double m_fRight;
		double m_bLeft;
		double m_bRight;
		DriveVelocity(double fLeft, double fRight, double bLeft, double bRight);
		static Kinematics::DriveVelocity inverseKinematics(RigidTransform2D::Delta velocity);
	};

	static RigidTransform2D::Delta forwardKinematics(double frDriveDelta, double flDriveDelta, double brDriveDelta, double blDriveDelta,
		double frRotationDelta, double flRotationDelta, double brRotationDelta, double blRotationDelta);

	static RigidTransform2D::Delta forwardKinematics(double frDriveDelta, double flDriveDelta, double brDriveDelta, double blDriveDelta,
		double frRotationDelta, double flRotationDelta, double brRotationDelta, double blRotationDelta, double gyroDelta);

	static RigidTransform2D integrateForwardKinematics(RigidTransform2D currentPose, double frDriveDelta, double flDriveDelta, double brDriveDelta, double blDriveDelta,
		double frRotationDelta, double flRotationDelta, double brRotationDelta, double blRotationDelta, Rotation2D currentHeading);

};
