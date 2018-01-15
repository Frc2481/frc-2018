#pragma once
#include "utils/Rotation2D.h"
#include "utils/RigidTransform2D.h"
#include "math.h"
#include "pathfinder.h"

class AimingParameters /*:public Subsystems*/{
private:
	RigidTransform2D m_transform;
	int m_trackID;
public:
	AimingParameters(RigidTransform2D targetTransform, int trackID);
	double getDistance();
	Rotation2D getTargetAngle();
	double getRobotAngle();
	int getTrackID();
};
