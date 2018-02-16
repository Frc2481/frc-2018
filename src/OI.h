#ifndef OI_H
#define OI_H

	class Joystick2481;

#include <Buttons/JoystickButton.h>
#include <Components/ComboButton.h>
#include "Components/AnalogJoystickButton.h"

class OI {
public:
	OI();
	Joystick2481* m_driverStick;
	Joystick2481* m_operatorStick;
	Joystick2481* GetDriverStick();
	Joystick2481* GetOperatorStick();

	Button* m_aDriverButton;
	Button* m_bDriverButton;
	Button* m_xDriverButton;
	Button* m_yDriverButton;

	Button* m_leftDriverBumper;
	Button* m_rightDriverBumper;

	Button* m_aOpButton;
	Button* m_bOpButton;
	Button* m_xOpButton;
	Button* m_yOpButton;

	Button* m_leftOpBumper;
	Button* m_rightOpBumper;

//driver
//	Button* m_shifterButton;
	Button* m_shifterButton;
	Button* m_fieldCentricButton;
	AnalogJoystickButton* m_intakeCubeButton;
	AnalogJoystickButton* m_climberButton;
	Button* m_armToIntakeFront;
	Button* m_armToIntakeBack;
	Button* m_armToIntake2Front;
	Button* m_armToIntake2Back;
	Button* m_releaseCubeButton;
	Button* m_stowButton;

//for testing
	AnalogJoystickButton* m_intakeCubeManualButton;
	Button* m_openClampButton;

//operator
	AnalogJoystickButton* m_extenderButton;
	AnalogJoystickButton* m_retractButton;
	AnalogJoystickButton* m_pivotForwardButton;
	AnalogJoystickButton* m_pivotReverseButton;
	Button* m_intakeButton;
	Button* m_releaseButton;

	Button* m_armToMiddleScaleFront;
	Button* m_armToMiddleScaleBack;
	Button* m_armToLowScaleFront;
	Button* m_armToLowScaleBack;
	Button* m_armToHighScaleFront;
	Button* m_armToHighScaleBack;

};

#endif  // OI_H
