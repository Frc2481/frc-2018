
#include <Commands/ArmExtendCommand.h>
#include <Commands/ArmPivotForwardCommand.h>
#include <Commands/ArmPivotReverseCommand.h>
#include <Commands/ArmRetractCommand.h>
#include <Commands/IntakeAcquireCubeCommandGroup.h>
#include <commands/IntakeGrabCubeCommandGroup.h>
#include <Commands/IntakeReleaseCubeCommandGroup.h>
#include <Commands/IntakeStopCubeCommandGroup.h>
#include <Commands/DriveTrainFarWinchCommand.h>
#include <Commands/DriveTrainNearWinchCommand.h>
#include "OI.h"
#include <WPILib.h>
#include "Components/Joystick2481.h"
#include "Commands/DriveTrainShiftCommand.h"
#include "XboxController.h"
#include "Commands/ArmBaseCommand.h"

OI::OI() {
	// Process operator interface input here.
	m_driverStick = new Joystick2481(0);
	m_operatorStick = new Joystick2481(1);

	m_shifterButton = new JoystickButton(m_driverStick, XB_A_BUTTON);
	m_shifterButton->WhileHeld(new DriveTrainShiftCommand());
//
//	m_extenderButton = new JoystickButton(m_driverStick, XB_A_BUTTON);
//	m_extenderButton->WhileHeld(new ArmExtendCommand());
//
//	m_retractButton = new JoystickButton(m_driverStick, XB_B_BUTTON);
//	m_retractButton->WhileHeld(new ArmRetractCommand());

	m_pivotForwardButton = new JoystickButton(m_driverStick, XB_X_BUTTON);
	m_pivotForwardButton->WhileHeld(new ArmPivotForwardCommand());

	m_pivotReverseButton = new JoystickButton(m_driverStick, XB_Y_BUTTON);
	m_pivotReverseButton->WhileHeld(new ArmPivotReverseCommand());
//
//	m_stowButton = new JoystickButton(m_driverStick, XB_DPAD_TOP);
//	m_stowButton->WhenPressed(new ArmToStow(""));
//
//	m_scaleFrontButton = new JoystickButton(m_driverStick, XB_DPAD_RIGHT);
//	m_scaleFrontButton->WhenPressed(new ArmToMidScaleFront(""));
//
//	m_scaleBackButton = new JoystickButton(m_driverStick, XB_DPAD_LEFT);
//	m_scaleBackButton->WhenPressed(new ArmToMidScaleBack(""));
//
//	m_intakeButton = new JoystickButton(m_driverStick, XB_LEFT_BUMPER);
//	m_intakeButton->WhenPressed(new IntakeAcquireCubeCommandGroup());
//
	m_releaseCubeButton = new JoystickButton(m_driverStick, XB_RIGHT_BUMPER);
	m_releaseCubeButton->WhenPressed(new IntakeReleaseCubeCommandGroup());

	m_intakeCubeButton = new JoystickButton(m_driverStick, XB_LEFT_BUMPER);
	m_intakeCubeButton->WhenPressed(new IntakeGrabCubeCommandGroup());
	m_intakeCubeButton->WhenReleased(new IntakeStopCubeCommandGroup());

	m_nearWinchButton = new JoystickButton(m_driverStick, XB_BACK_BUTTON);
	m_nearWinchButton->WhileHeld(new DriveTrainNearWinchCommand());

	m_farWinchButton = new JoystickButton(m_driverStick, XB_START_BUTTON);
	m_farWinchButton->WhileHeld(new DriveTrainFarWinchCommand());
}

Joystick2481* OI::GetDriverStick() {
	return m_driverStick;
}

Joystick2481* OI::GetOperatorStick() {
	return m_operatorStick;
}
