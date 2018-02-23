
#ifndef _POV_JOYSTICK_BUTTON_H__
#define _POV_JOYSTICK_BUTTON_H__

#include "WPILib.h"

class POVJoystickButton : public Button
{
public:
	POVJoystickButton(GenericHID *joystick, uint32_t POVNumber, int angle);
	virtual ~POVJoystickButton() {}

	virtual bool Get();

private:
	GenericHID *m_joystick;
	uint32_t m_POVNumber;
	int m_angle;
};

#endif
