#ifndef OI_H
#define OI_H

	class Joystick2481;

#include <Buttons/JoystickButton.h>

class OI {
public:
	OI();
	Joystick2481* m_driverStick;
	Joystick2481* m_operatorStick;
	Joystick2481* GetDriverStick();
	Joystick2481* GetOperatorStick();

	Button* m_shifterButton;
};

#endif  // OI_H
