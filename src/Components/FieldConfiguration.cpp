/*
 * FieldConfiguration.cpp
 *
 *  Created on: Jan 15, 2018
 *      Author: FIRSTMentor
 */

#include <string>
#include <FieldConfiguration.h>

FieldConfiguration::FieldConfiguration() {
	m_ourSwitchPlate = UNKNOWN;
	m_theirSwitchPlate=UNKNOWN;
	m_scalePlate = UNKNOWN;
}

FieldConfiguration::~FieldConfiguration() {
}

void FieldConfiguration::Initialize() {
	std::string gameData= DriverStation::GetInstance().GetGameSpecificMessage();
	m_ourSwitchPlate = LookupPlate(gameData[0]);
	m_theirSwitchPlate = LookupPlate(gameData[2]);
	m_scalePlate = LookupPlate(gameData[1]);
}

FieldConfiguration::Plate FieldConfiguration::GetOurSwitchPlate() {
	if (m_ourSwitchPlate == UNKNOWN) {
		Initialize();
	}
	return m_ourSwitchPlate;
}

FieldConfiguration::Plate FieldConfiguration::GetTheirSwitchPlate() {
	if (m_theirSwitchPlate == UNKNOWN) {
		Initialize();
	}
	return m_theirSwitchPlate;
}

FieldConfiguration::Plate FieldConfiguration::GetScalePlate() {
	if (m_scalePlate == UNKNOWN) {
		Initialize();
	}
	return m_scalePlate;
}

FieldConfiguration::Plate FieldConfiguration::LookupPlate(char c) {
	if (m_plateMap.find(c) != m_plateMap.end()){
		return m_plateMap[c];
	}
	else{
		return UNKNOWN;
	}
}
