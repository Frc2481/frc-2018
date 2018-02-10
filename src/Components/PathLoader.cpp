/*
 * PathLoader.cpp
 *
 *  Created on: Feb 6, 2018
 *      Author: Team2481
 */

#include <Components/PathLoader.h>
#include <sstream>

PathLoader::PathLoader(Path2D& path) : m_path(path) {
}

PathLoader::~PathLoader() {
}

void PathLoader::LoadPath(const std::string filePath) {
	//TODO error handling when no path is available

	std::ifstream fin(filePath);

	double x, y, yaw, time;
	std::string field;

	while(fin.good()) {
		std::getline(fin, field, ',');
		std::istringstream(field) >> x;

		std::getline(fin, field, ',');
		std::stringstream(field) >> y;

		std::getline(fin, field, ',');
		std::stringstream(field) >> yaw;

		std::getline(fin, field);
		std::stringstream(field) >> time;

		m_path.put(InterpolatingDouble(time), RigidTransform2D(Translation2D(x, y), Rotation2D::fromDegrees(yaw)));
	}
}
