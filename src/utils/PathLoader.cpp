/*
 * PathLoader.cpp
 *
 *  Created on: Feb 6, 2018
 *      Author: Team2481
 */

#include <utils/PathLoader.h>
#include <sstream>

PathLoader::PathLoader(Path2D& path) : m_path(path) {
}

PathLoader::~PathLoader() {
}

void PathLoader::LoadPath(const std::string filePath) {
	//TODO error handling when no path is available

	PathPoint2D pathPoint;

	std::ifstream fin(filePath);

	std::string field;

	while(fin.good()) {
		std::getline(fin, field, ',');
		std::istringstream(field) >> pathPoint.time;

		std::getline(fin, field, ',');
		std::stringstream(field) >> pathPoint.xPos;

		std::getline(fin, field, ',');
		std::stringstream(field) >> pathPoint.yPos;

		std::getline(fin, field, ',');
		std::stringstream(field) >> pathPoint.yaw;

		std::getline(fin, field, ',');
		std::stringstream(field) >> pathPoint.xVel;

		std::getline(fin, field, ',');
		std::stringstream(field) >> pathPoint.yVel;

		std::getline(fin, field, ',');
		std::stringstream(field) >> pathPoint.yawVel;

		std::getline(fin, field, ',');
		std::stringstream(field) >> pathPoint.xAccel;

		std::getline(fin, field, ',');
		std::stringstream(field) >> pathPoint.yAccel;

		std::getline(fin, field);
		std::stringstream(field) >> pathPoint.yawAccel;

		m_path.push_back(pathPoint);
	}
}
