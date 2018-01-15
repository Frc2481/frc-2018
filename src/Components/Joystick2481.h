/*
 * Joystick2481.h
 *
 *  Created on: Jan 21, 2013
 *      Author: Team2481
 */

#ifndef JOYSTICK2481_H_
#define JOYSTICK2481_H_

#include "Joystick.h"

class Joystick2481 : public Joystick {
private:
	static char inputShape[255];
	double s;
	double m;
	double b;
	double t;
	double sTwist;
	double mTwist;
	double bTwist;
	double tTwist;
public:
	Joystick2481(int port);
	virtual ~Joystick2481();
	virtual float GetRawAxis(int axis);
	float GetRawAxisTwist(int axis);
};

#endif /* JOYSTICK2481_H_ */
