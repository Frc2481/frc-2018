#ifndef OI_H
#define OI_H

	class Joystick2481;

#include "frc/Buttons/JoystickButton.h"
#include "Components/ComboButton.h"
#include "Components/AnalogJoystickButton.h"
#include "Components/POVJoystickButton.h"

class OI {
public:
	OI();
	Joystick2481* m_driverStick;
	Joystick2481* m_operatorStick;
	Joystick2481* m_climbStick;
	Joystick2481* GetDriverStick();
	Joystick2481* GetOperatorStick();

	frc::Button* m_aOpButton;
	frc::Button* m_bOpButton;
	frc::Button* m_xOpButton;
	frc::Button* m_yOpButton;

	frc::Button* m_leftOpBumper;
	frc::Button* m_rightOpBumper;

	frc::Button* m_startButton;
	frc::Button* m_backButton;

	frc::Button* m_leftTrigger;
	frc::Button* m_rightTrigger;

//driver
	frc::Button* m_shifterButton;
	frc::Button* m_fieldCentricButton;
	AnalogJoystickButton* m_intakeCubeButton;
//	AnalogJoystickButton* m_climberButton;
	frc::Button* m_armToIntakeFront;
	frc::Button* m_armToIntakeBack;
	frc::Button* m_stowButtonDriver;
	frc::Button* m_intake3PosButton;
	frc::Button* m_driveTrainZeroGyro;

//for testing
//	AnalogJoystickButton* m_intakeCubeManualButton;
	frc::Button* m_manualExtend;
	frc::Button* m_manualRetract;

//operator
	AnalogJoystickButton* m_extenderButton;
	AnalogJoystickButton* m_retractButton;
	AnalogJoystickButton* m_pivotUpButton;
	AnalogJoystickButton* m_pivotDownButton;
//	frc::Button* m_intakeButton;
//	frc::Button* m_releaseButton;


	frc::Button* m_armToSwitchFront;
	frc::Button* m_armToSwitchBack;
	frc::Button* m_armToLowScaleFront;
	frc::Button* m_armToLowScaleBack;
	frc::Button* m_armToMiddleScaleFront;
	frc::Button* m_armToMiddleScaleBack;
	frc::Button* m_armToHighScaleFront;
	frc::Button* m_armToHighScaleBack;

	frc::Button* m_nextPos;
	frc::Button* m_lastPos;

	frc::Button* m_mirrorArmPos;

//	frc::Button* m_deployRampsPTOButton;
//	frc::Button* m_climbButton;

	frc::Button* m_outerWinch;
	frc::Button* m_innerWinch;
	frc::Button* m_extensionOpenLoop;

	frc::Button* m_stowButtonOp;

	frc::Button* m_exchangeFrontButton;
	frc::Button* m_exchangeBackButton;

	frc::Button* m_variableReleaseSpeed;
};

#endif  // OI_H
