/*
 * ObserverPIDSourceY.cpp
 *
 *  Created on: Jan 26, 2018
 *      Author: Team2481
 */

#include <Components/ObserverPIDSourceY.h>

ObserverPIDSourceY::ObserverPIDSourceY(Observer* src) {
	obj = src;
}

ObserverPIDSourceY::~ObserverPIDSourceY() {
	// TODO Auto-generated destructor stub
}

double ObserverPIDSourceY::PIDGet() {
	return obj->GetLastRobotPose().getTranslation().getY();
}
