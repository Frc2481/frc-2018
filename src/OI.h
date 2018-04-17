#ifndef OI_H
#define OI_H

	class Joystick2481;

#include <Buttons/JoystickButton.h>
#include <Components/ComboButton.h>
#include "Components/AnalogJoystickButton.h"
#include <Components/POVJoystickButton.h>

class OI {
public:
	OI();
	Joystick2481* m_driverStick;
	Joystick2481* m_operatorStick;
	Joystick2481* m_climbStick;
	Joystick2481* GetDriverStick();
	Joystick2481* GetOperatorStick();

	Button* m_aOpButton;
	Button* m_bOpButton;
	Button* m_xOpButton;
	Button* m_yOpButton;

	Button* m_leftOpBumper;
	Button* m_rightOpBumper;

	Button* m_startButton;
	Button* m_backButton;

	Button* m_leftTrigger;
	Button* m_rightTrigger;

//driver
	Button* m_shifterButton;
	Button* m_fieldCentricButton;
	AnalogJoystickButton* m_intakeCubeButton;
//	AnalogJoystickButton* m_climberButton;
	Button* m_armToIntakeFront;
	Button* m_armToIntakeBack;
	Button* m_stowButtonDriver;
	Button* m_intake3PosButton;
	Button* m_driveTrainZeroGyro;

//for testing
//	AnalogJoystickButton* m_intakeCubeManualButton;
	Button* m_manualExtend;
	Button* m_manualRetract;

//operator
	AnalogJoystickButton* m_extenderButton;
	AnalogJoystickButton* m_retractButton;
	AnalogJoystickButton* m_pivotUpButton;
	AnalogJoystickButton* m_pivotDownButton;
//	Button* m_intakeButton;
//	Button* m_releaseButton;


	Button* m_armToSwitchFront;
	Button* m_armToSwitchBack;
	Button* m_armToLowScaleFront;
	Button* m_armToLowScaleBack;
	Button* m_armToMiddleScaleFront;
	Button* m_armToMiddleScaleBack;
	Button* m_armToHighScaleFront;
	Button* m_armToHighScaleBack;

	Button* m_nextPos;
	Button* m_lastPos;

	Button* m_mirrorArmPos;

//	Button* m_deployRampsPTOButton;
//	Button* m_climbButton;

	Button* m_outerWinch;
	Button* m_innerWinch;
	Button* m_extensionOpenLoop;
	Button* m_engagePto;

	Button* m_stowButtonOp;

	Button* m_exchangeFrontButton;
	Button* m_exchangeBackButton;

	Button* m_variableReleaseSpeed;
};

#endif  // OI_H
