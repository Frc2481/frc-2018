/*
 * LimeLight.cpp
 *
 *  Created on: Jan 18, 2018
 *      Author: Team2481
 */

#include "WPILib.h"
#include <Subsystems/LimeLight.h>

LimeLight::LimeLight() {
	// TODO Auto-generated constructor stub
}

LimeLight::~LimeLight() {
	// TODO Auto-generated destructor stub
}

//double LimeLight::GetValues() {
////	return targetOffsetAngle_Horizontal = table->GetNumber("tx");
//}

//double LimeLight::EstimateDistance() {
//}

float LimeLight::targetOffsetAngle_Horizontal() {
	std::shared_ptr<NetworkTable> table =   NetworkTable::GetTable("limelight");
	table->GetNumber("tx", 0);
}

float LimeLight::targetOffsetAngle_Vertical() {
	std::shared_ptr<NetworkTable> table =   NetworkTable::GetTable("limelight");
	table->GetNumber("ty", 0);
}

float LimeLight::targetArea() {
	std::shared_ptr<NetworkTable> table =   NetworkTable::GetTable("limelight");
	table->GetNumber("ta", 0);
}

float LimeLight::targetSkew() {
	std::shared_ptr<NetworkTable> table =   NetworkTable::GetTable("limelight");
	table->GetNumber("ty", 0);
}

void LimeLight::TurnOnLED(){
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
	table->PutNumber("ledMode", 0);
}

void LimeLight::TurnOffLED(){
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
	table->PutNumber("ledMode", 1);
}

void LimeLight::BlinkLight(){
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
	table->PutNumber("ledMode", 2);
}

void LimeLight::ActivateRedScale(){
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
	table->PutNumber("pipeline", 0);
}

void LimeLight::ActivateBlueScale(){
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
	table->PutNumber("pipeline", 1);
}

void LimeLight::PowerCubePipeline(){
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
	table->PutNumber("pipeline", 2);
}

void LimeLight::ActivateFencePipeline(){
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("limelight");
	table->PutNumber("pipeline", 3);
}
//std::shared_ptr<NetworkTable> table =   NetworkTable::GetTable("limelight");
//float targetOffsetAngle_Horizontal = table->GetNumber("tx");
//float targetOffsetAngle_Vertical = table->GetNumber("ty");
//float targetArea = table->GetNumber("ta");
//float targetSkew = table->GetNumber("ts");
