/*
 * AnalogJoystickButton.h
 *
 *  Created on: Feb 3, 2017
 *      Author: FIRSTMentor
 */

#ifndef _ANALOG_JOYSTICK_BUTTON_H__
#define _ANALOG_JOYSTICK_BUTTON_H__

#include "WPILib.h"

class AnalogJoystickButton : public Button
{
public:
	AnalogJoystickButton(GenericHID *joystick, int axisNumber, float threshold);
	virtual ~AnalogJoystickButton() {}

	virtual bool Get();

private:
	GenericHID *m_joystick;
	int m_axisNumber;
	float m_threshold;
};

#endif
