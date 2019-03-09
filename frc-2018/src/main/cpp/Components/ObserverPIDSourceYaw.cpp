/*
 * ObserverPIDSourceYaw.cpp
 *
 *  Created on: Jan 26, 2018
 *      Author: Team2481
 */

#include "Components/ObserverPIDSourceYaw.h"

ObserverPIDSourceYaw::ObserverPIDSourceYaw(Observer* src) {
	obj = src;
}

ObserverPIDSourceYaw::~ObserverPIDSourceYaw() {
	// TODO Auto-generated destructor stub
}

double ObserverPIDSourceYaw::PIDGet() {
	return obj->GetLastRobotPose().getRotation().getDegrees();
}
