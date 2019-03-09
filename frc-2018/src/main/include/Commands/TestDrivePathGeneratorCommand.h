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

class TestDrivePathGeneratorCommand : public frc::InstantCommand{
private:
	std::vector<Waypoint> m_waypoints;
	DrivePathGenerator drivePathGen;
public:
	TestDrivePathGeneratorCommand() : InstantCommand("TestDrivePathGeneratorCommand") {

	}

	void Initialize() {
		Waypoint waypoint;
		waypoint.pose = RigidTransform2D(Translation2D(0, 0), Rotation2D::fromDegrees(0));
		waypoint.maxDistAway = 0;
		m_waypoints.push_back(waypoint);

		waypoint.pose = RigidTransform2D(Translation2D(25, 50), Rotation2D::fromDegrees(45));
		waypoint.maxDistAway = 0;
		m_waypoints.push_back(waypoint);

		waypoint.pose = RigidTransform2D(Translation2D(75, 25), Rotation2D::fromDegrees(120));
		waypoint.maxDistAway = 30;
		m_waypoints.push_back(waypoint);

		waypoint.pose = RigidTransform2D(Translation2D(150, 100), Rotation2D::fromDegrees(45));
		waypoint.maxDistAway = 50;
		m_waypoints.push_back(waypoint);

		waypoint.pose = RigidTransform2D(Translation2D(200, 120), Rotation2D::fromDegrees(-30));
		waypoint.maxDistAway = 10000;
		m_waypoints.push_back(waypoint);

		waypoint.pose = RigidTransform2D(Translation2D(75, 150), Rotation2D::fromDegrees(0));
		waypoint.maxDistAway = 0;
		m_waypoints.push_back(waypoint);

		drivePathGen.GeneratePath(m_waypoints, 100, 100, 10.0);
	}
	virtual ~TestDrivePathGeneratorCommand() {}
};

#endif /* SRC_COMMANDS_TESTDRIVEPATHGENERATORCOMMAND_H_ */
