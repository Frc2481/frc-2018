/*
 * DriveControllerOutput.cpp
 *
 *  Created on: Jan 26, 2018
 *      Author: Team2481
 */

#include "Components/DriveControllerOutput.h"

DriveControllerOutput::DriveControllerOutput() {
	// TODO Auto-generated constructor stub

}

DriveControllerOutput::~DriveControllerOutput() {
	// TODO Auto-generated destructor stub
}

void DriveControllerOutput::PIDWrite(double output) {
	val = output;
}

double DriveControllerOutput::GetOutput() {
	return val;
}
