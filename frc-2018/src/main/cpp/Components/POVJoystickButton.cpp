
#include "Components/POVJoystickButton.h"

POVJoystickButton::POVJoystickButton(frc::GenericHID *joystick, uint32_t POVNumber, int angle ) :
	m_joystick(joystick),
	m_POVNumber(POVNumber),
	m_angle(angle){
}

bool POVJoystickButton::Get()
{
	return m_joystick->GetPOV(m_POVNumber) == m_angle;
}
