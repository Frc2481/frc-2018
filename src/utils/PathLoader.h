/*
 * PathLoader.h
 *
 *  Created on: Feb 6, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMPONENTS_PATHLOADER_H_
#define SRC_COMPONENTS_PATHLOADER_H_

#include <fstream>
#include <vector>

#include <utils/RigidTransform2D.h>
#include <utils/InterpolatingMap.h>
#include <utils/InterpolatingDouble.h>

struct PathPoint2D {
	double time;
	double xPos;
	double xVel;
	double xAccel;
	double yPos;
	double yVel;
	double yAccel;
	double yaw;
	double yawVel;
	double yawAccel;
};

typedef std::vector<PathPoint2D> Path2D;

class PathLoader {
public:
	PathLoader(Path2D& path);
	virtual ~PathLoader();

	void LoadPath(const std::string filePath);

private:
	Path2D& m_path;
};

#endif /* SRC_COMPONENTS_PATHLOADER_H_ */
