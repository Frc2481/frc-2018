/*
 * ObserverPIDSourceX.cpp
 *
 *  Created on: Jan 26, 2018
 *      Author: Team2481
 */

#include "Components/ObserverPIDSourceX.h"

ObserverPIDSourceX::ObserverPIDSourceX(Observer* src) {
	obj = src;
}

ObserverPIDSourceX::~ObserverPIDSourceX() {
	// TODO Auto-generated destructor stub
}

double ObserverPIDSourceX::PIDGet() {
	return obj->GetLastRobotPose().getTranslation().getX();
}
