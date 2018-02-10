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
	Button* m_extenderButton;
	Button* m_retractButton;
	Button* m_intakeButton;
	Button* m_pivotForwardButton;
	Button* m_pivotReverseButton;
	Button* m_stowButton;
	Button* m_scaleFrontButton;
	Button* m_scaleBackButton;
	Button* m_releaseCubeButton;
	Button* m_intakeCubeButton;

	Button* m_farWinchButton;
	Button* m_nearWinchButton;
};

#endif  // OI_H
