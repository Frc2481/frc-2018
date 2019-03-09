
#ifndef _POV_JOYSTICK_BUTTON_H__
#define _POV_JOYSTICK_BUTTON_H__

#include <frc/WPILib.h>

class POVJoystickButton : public frc::Button
{
public:
	POVJoystickButton(frc::GenericHID *joystick, uint32_t POVNumber, int angle);
	virtual ~POVJoystickButton() {}

	virtual bool Get();

private:
	frc::GenericHID *m_joystick;
	uint32_t m_POVNumber;
	int m_angle;
};

#endif
