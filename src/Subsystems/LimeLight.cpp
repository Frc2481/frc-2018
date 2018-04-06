/*
 * LimeLight.cpp
 *
 *  Created on: Jan 18, 2018
 *      Author: Team2481
 */

#include "WPILib.h"
#include <Subsystems/LimeLight.h>

LimeLight::LimeLight() : Subsystem("LimeLight") {
	// TODO Auto-generated constructor stub
}

LimeLight::~LimeLight() {
	// TODO Auto-generated destructor stub
}

double LimeLight::getPowerCubeTargetValid() {
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
	return table->GetNumber("tv", 0);
}

double LimeLight::getPowerCubeTargetOffsetAngleHorizontal() {
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
//	printf("x = %f\n", table->GetNumber("tx", 0));
	return table->GetNumber("tx", 0);
}

double LimeLight::getPowerCubeTargetOffsetAngleVertical() {
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
//	printf("y = %f\n", table->GetNumber("ty", 0));
	return table->GetNumber("ty", 0);
}

double LimeLight::getPowerCubeTargetArea() {
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
	return table->GetNumber("ta", 0);
}

double LimeLight::getPowerCubeTargetSkew() {
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
	return table->GetNumber("ts", 0);
}

void LimeLight::TurnOnLED() {
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
	table->PutNumber("ledMode", 0);
}

void LimeLight::TurnOffLED() {
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
	table->PutNumber("ledMode", 1);
}

void LimeLight::BlinkLED() {
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
	table->PutNumber("ledMode", 2);
}

void LimeLight::ActivatePowerCubePipeline() {
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
	table->PutNumber("pipeline", 0);
}

void LimeLight::CalculatePowerCubePose() {
	// Calculations in Robot Frame
	// +x = robot right
	// +y = robot forward
	// +z = robot up
	// +yaw = CCW, zero is robot forward

	double xCube = 0;
	double yCube = 0;
	double yawCube = 0;

	if(getPowerCubeTargetValid()) {
		yawCube = getPowerCubeTargetOffsetAngleHorizontal() + RobotParameters::cameraOffsetYaw;

		double tanVal = tan((-getPowerCubeTargetOffsetAngleVertical() - RobotParameters::cameraOffsetPitch) * M_PI / 180.0);
		if (tanVal) {
			yCube = (RobotParameters::cameraOffsetZ - (RobotParameters::cubeHeight / 2.0)) / tanVal;
			xCube = RobotParameters::cameraOffsetX + (yCube - RobotParameters::cameraOffsetY) * tan(yawCube * M_PI / 180.0);
		}
	}

	Translation2D cubePos = Translation2D(xCube, yCube);
	Rotation2D cubeRot = Rotation2D::fromDegrees(yawCube);

	SetPowerCubePose(RigidTransform2D(cubePos, cubeRot));

	SmartDashboard::PutNumber("x Cube", xCube);
	SmartDashboard::PutNumber("y Cube", yCube);
	SmartDashboard::PutNumber("yaw Cube", yawCube);
}

void LimeLight::SetPowerCubePose(RigidTransform2D cubePose) {
	m_powerCubePose = cubePose;
}

RigidTransform2D LimeLight::GetPowerCubePose() {
	return m_powerCubePose;
}

//void LimeLight::ActivateRedScalePipeline() {
//	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
//	table->PutNumber("pipeline", 1);
//}

//void LimeLight::ActivateBlueScalePipeline() {
//	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
//	table->PutNumber("pipeline", 2);
//}

//void LimeLight::ActivateFencePipeline() {
//	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
//	table->PutNumber("pipeline", 3);
//}

void LimeLight::Periodic() {
	CalculatePowerCubePose();
}

