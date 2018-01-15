#include "AimingParameters.h"

AimingParameters::AimingParameters(RigidTransform2D targetTransform, int trackID) {
	m_transform = targetTransform;
	m_trackID = trackID;
}

double AimingParameters::getDistance() {
	return m_transform.getTranslation().norm();
}

Rotation2D AimingParameters::getTargetAngle() {
	return m_transform.inverse().getRotation();
}

double AimingParameters::getRobotAngle() {
	double xParam =  m_transform.getTranslation().getX();
	double yParam = m_transform.getTranslation().getY();
	double angle = atan(yParam/xParam);
	return r2d(angle);
}

int AimingParameters::getTrackID() {
	return m_trackID;
}
