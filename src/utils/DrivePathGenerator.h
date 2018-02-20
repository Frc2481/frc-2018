/*
 * DrivePathGenerator.h
 *
 *  Created on: Feb 12, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_UTILS_DRIVEPATHGENERATOR_H_
#define SRC_UTILS_DRIVEPATHGENERATOR_H_

#include "utils/RigidTransform2D.h"
#include <vector>
#include <math.h>
#include <WPILib.h>
#include <utils/InterpolatingMap.h>
#include <utils/InterpolatingDouble.h>
#include <fstream>
#include <iostream>

struct Waypoint {
	RigidTransform2D pose;
	double maxDistAway;
};

struct TempWaypoint {
	RigidTransform2D pose;
	double maxDistAway;
	int tempPathIndex;
	double distTraveled;
	int finalPathIndex;
};

struct FinalPath {
	RigidTransform2D pose;
	double time;
};

class DrivePathGenerator {
public:
	DrivePathGenerator();
	virtual ~DrivePathGenerator();
	void GeneratePath(std::vector<Waypoint> &waypoints, double maxSpeed, double maxAccel, double sampleRate);
};

#endif /* SRC_UTILS_DRIVEPATHGENERATOR_H_ */
