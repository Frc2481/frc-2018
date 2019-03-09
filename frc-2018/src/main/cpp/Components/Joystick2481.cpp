/*
 * Joystick2481.cpp
 *
 *  Created on: Jan 21, 2013
 *      Author: Team2481
 */

#include "Components/Joystick2481.h"
#include <math.h>

Joystick2481::Joystick2481(int port) : Joystick(port) {
	s = 0.57;
	m = 1.340508;
	t = 0.381021;
	b = -0.340508;
	sTwist = 0.75;
	mTwist = 1.56538;
	tTwist = 0.54176;
	bTwist = -0.56538;
}

Joystick2481::~Joystick2481() {
	// TODO Auto-generated destructor stub
}

float Joystick2481::GetRawAxis(int axis) {
	float x = Joystick::GetRawAxis(axis);
	float scale = 1.0f; //boost ? 1.0f : 0.5f;

    if (x < -t){
        return scale * (m * x - b);
    }
    if (x > -t && x < t){
        return scale * (1 / (pow(s,2.0)) * pow(x,3.0));
    }
    else{
        return scale * (m * x + b);
    }

}

float Joystick2481::GetRawAxisTwist(int axis) {
	float x = Joystick::GetRawAxis(axis);
	float scale = 1.0f; //boost ? 1.0f : 0.5f;

	if (x < -tTwist){
	    return scale * (mTwist * x - bTwist);
	}
	if (x > -tTwist && x < tTwist){
	    return scale * (1 / (pow(sTwist,2.0)) * pow(x,3.0));
	}
	else{
	   return scale * (mTwist * x + bTwist);
	}
}
