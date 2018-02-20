/*
 * AnalogJoystickButton.cpp
 *
 *  Created on: Feb 3, 2017
 *      Author: FIRSTMentor
 */

#include "AnalogJoystickButton.h"

AnalogJoystickButton::AnalogJoystickButton(GenericHID *joystick, int axisNumber, float threshold ) {
	m_threshold = threshold;
	m_joystick = joystick;
	m_axisNumber = axisNumber;
}

bool AnalogJoystickButton::Get()
{
	if(m_threshold < 0)
		return m_joystick->GetRawAxis((uint32_t)m_axisNumber) < m_threshold;
	else if(m_threshold > 0)
		return m_joystick->GetRawAxis((uint32_t)m_axisNumber) > m_threshold;
	return false;

}
