/*
 * TestDrivePathGeneratorCommand.h
 *
 *  Created on: Feb 13, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_TESTDRIVEPATHGENERATORCOMMAND_H_
#define SRC_COMMANDS_TESTDRIVEPATHGENERATORCOMMAND_H_

#include "CommandBase.h"
#include "utils/DrivePathGenerator.h"

class TestDrivePathGeneratorCommand : public InstantCommand{
private:
	std::vector<Waypoint> m_waypoints;
public:
	TestDrivePathGeneratorCommand() : InstantCommand("TestDrivePathGeneratorCommand") {

	}

	void Initialize() {
		Waypoint waypoint;
		waypoint.pose = RigidTransform2D(Translation2D(0, 0), Rotation2D::fromDegrees(0));
		waypoint.maxDistAway = 0;
		m_waypoints.push_back(waypoint);

		waypoint.pose = RigidTransform2D(Translation2D(0, 100), Rotation2D::fromDegrees(90));
		waypoint.maxDistAway = 20;
		m_waypoints.push_back(waypoint);

		waypoint.pose = RigidTransform2D(Translation2D(50, 100), Rotation2D::fromDegrees(180));
		waypoint.maxDistAway = 20;
		m_waypoints.push_back(waypoint);

		waypoint.pose = RigidTransform2D(Translation2D(50, 150), Rotation2D::fromDegrees(-90));
		waypoint.maxDistAway = 20;
		m_waypoints.push_back(waypoint);

		waypoint.pose = RigidTransform2D(Translation2D(75, 200), Rotation2D::fromDegrees(0));
		waypoint.maxDistAway = 0;
		m_waypoints.push_back(waypoint);
	}
	virtual ~TestDrivePathGeneratorCommand() {}
};

#endif /* SRC_COMMANDS_TESTDRIVEPATHGENERATORCOMMAND_H_ */
