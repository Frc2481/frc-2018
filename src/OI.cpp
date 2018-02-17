
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

//driver
	m_aDriverButton = new JoystickButton(m_driverStick, XB_A_BUTTON);
	m_bDriverButton = new JoystickButton(m_driverStick, XB_B_BUTTON);
	m_xDriverButton = new JoystickButton(m_driverStick, XB_X_BUTTON);
	m_yDriverButton = new JoystickButton(m_driverStick, XB_Y_BUTTON);

	m_leftDriverBumper = new JoystickButton(m_driverStick, XB_LEFT_BUMPER);
	m_rightDriverBumper = new JoystickButton(m_driverStick, XB_RIGHT_BUMPER);

//operator
	m_aOpButton = new JoystickButton(m_operatorStick, XB_A_BUTTON);
	m_bOpButton = new JoystickButton(m_operatorStick, XB_B_BUTTON);
	m_xOpButton = new JoystickButton(m_operatorStick, XB_X_BUTTON);
	m_yOpButton = new JoystickButton(m_operatorStick, XB_Y_BUTTON);

	m_leftOpBumper = new JoystickButton(m_operatorStick, XB_LEFT_BUMPER);
	m_rightOpBumper = new JoystickButton(m_operatorStick, XB_RIGHT_BUMPER);



//driver
//	m_shifterButton = new JoystickButton(m_driverStick, XB_RIGHT_BUMPER);
//	m_shifterButton->WhileHeld(new DriveTrainShiftCommand());

	m_shifterButton = new JoystickButton(m_driverStick, XB_RIGHT_BUMPER);
	m_shifterButton->WhileHeld(new DriveTrainShiftCommand());

//	m_fieldCentricButton = new JoystickButton(m_driverStick, XB_LEFT_BUMPER);
//	m_fieldCentricButton->WhileHeld(new DriveTrainFieldCentricCommand());

	m_intakeCubeButton = new AnalogJoystickButton(m_driverStick, XB_RIGHT_TRIGGER, 0.5);
	m_intakeCubeButton->WhenPressed(new IntakeAcquireCubeCommandGroup());

//	m_climberButton = new AnalogJoystickButton(m_driverStick, XB_LEFT_TRIGGER, 0.5);
//	m_climberButton->ToggleWhenPressed(new ClimberExtendCommand()); //figure out to toggle
//	m_climberButton->ToggleWhenPressed(new CLimberRetractCommand()); //figure out to toggle



//	m_armToIntakeFront = new JoystickButton(m_driverStick, XB_A_BUTTON);
//	m_armToIntakeFront->WhenPressed(new ArmToIntakeFront(""));
//
//	m_armToIntakeBack = new JoystickButton(m_driverStick, XB_X_BUTTON);
//	m_armToIntakeBack->WhenPressed(new ArmToIntakeBack(""));
//
//	m_releaseCubeButton = new JoystickButton(m_driverStick, XB_B_BUTTON);
//	m_releaseCubeButton->WhileHeld(new IntakeReleaseCubeCommandGroup());
//	m_releaseCubeButton->WhenReleased(new IntakeRollerOffCommand());
//
//	m_stowButton = new JoystickButton(m_driverStick, XB_Y_BUTTON);
//	m_stowButton->WhenPressed(new ArmToStow(""));



	m_armToIntakeFront = new ComboButton(m_aDriverButton, m_leftDriverBumper, false);
	m_armToIntakeFront->WhenPressed(new ArmToIntakeFront(""));

	m_armToIntakeBack = new ComboButton(m_xDriverButton, m_leftDriverBumper, false);
	m_armToIntakeBack->WhenPressed(new ArmToIntakeBack(""));

	m_armToIntake2Front = new ComboButton(m_aDriverButton, m_leftDriverBumper, true);
	m_armToIntake2Front->WhenPressed(new ArmToIntake2Front(""));

	m_armToIntake2Back = new ComboButton(m_xDriverButton, m_leftDriverBumper, true);
	m_armToIntake2Back->WhenPressed(new ArmToIntake2Back(""));

	m_releaseCubeButton = new ComboButton(m_bDriverButton, m_leftDriverBumper, false);
	m_releaseCubeButton->WhileHeld(new IntakeReleaseCubeCommandGroup(0.5));
	m_releaseCubeButton->WhenReleased(new IntakeRollerOffCommand());

	m_stowButton = new ComboButton(m_yDriverButton, m_leftDriverBumper, false);
	m_stowButton->WhenPressed(new ArmToStow(""));




//extra
//	m_intakeCubeManualButton = new AnalogJoystickButton(m_driverStick, XB_RIGHT_TRIGGER, .5);
//	m_intakeCubeManualButton->WhileHeld(new IntakeGrabCubeCommandGroup());
//	m_intakeCubeManualButton->WhenReleased(new IntakeStopCubeCommandGroup());


//	m_nearWinchButton = new JoystickButton(m_driverStick, XB_BACK_BUTTON);
//	m_nearWinchButton->WhileHeld(new DriveTrainNearWinchCommand());
//
//	m_farWinchButton = new JoystickButton(m_driverStick, XB_START_BUTTON);
//	m_farWinchButton->WhileHeld(new DriveTrainFarWinchCommand());
//
//	m_armToHighScale = new JoystickButton(m_driverStick, XB_Y_BUTTON);
//	m_armToHighScale->WhenPressed(new ArmToHighScaleFront(""));
//
//	m_armToMidScale = new JoystickButton(m_driverStick, XB_B_BUTTON);
//	m_armToMidScale->WhenPressed(new ArmToMidScaleFront(""));

//operator

	m_extenderButton = new AnalogJoystickButton(m_operatorStick, XB_LEFT_Y_AXIS, 0.25); //left stick up
	m_extenderButton->WhileHeld(new ArmExtendCommand());

	m_retractButton = new AnalogJoystickButton(m_operatorStick, XB_LEFT_Y_AXIS, -0.25); //left stick down
	m_retractButton->WhileHeld(new ArmRetractCommand());

	m_pivotForwardButton = new AnalogJoystickButton(m_operatorStick, XB_RIGHT_Y_AXIS, 0.25); // right stick up
	m_pivotForwardButton->WhileHeld(new ArmPivotForwardCommand());

	m_pivotReverseButton = new AnalogJoystickButton(m_operatorStick, XB_RIGHT_Y_AXIS, -0.25); //right stick down
	m_pivotReverseButton->WhileHeld(new ArmPivotReverseCommand());


	m_intakeButton = new AnalogJoystickButton(m_operatorStick, XB_RIGHT_TRIGGER, 0.5);
	m_intakeButton->WhileHeld(new IntakeGrabCubeCommandGroup());
	m_intakeButton->WhenReleased(new IntakeStopCubeCommandGroup());

	m_releaseButton = new AnalogJoystickButton(m_operatorStick, XB_LEFT_TRIGGER, 0.5);
	m_releaseButton->WhenPressed(new IntakeReleaseCubeCommandGroup(1.0));


	//arm positions
	m_armToLowScaleFront = new ComboButton(m_bOpButton, m_leftOpBumper, false);
	m_armToLowScaleFront->WhenPressed(new ArmToLowScaleFront(""));

	m_armToMiddleScaleFront = new ComboButton(m_xOpButton, m_leftOpBumper, false);
	m_armToMiddleScaleFront->WhenPressed(new ArmToMidScaleFront(""));

	m_armToHighScaleFront = new ComboButton(m_yOpButton, m_leftOpBumper, false);
	m_armToHighScaleFront->WhenPressed(new ArmToHighScaleFront(""));


	m_armToLowScaleBack = new ComboButton(m_bOpButton, m_leftOpBumper, true);
	m_armToLowScaleBack->WhenPressed(new ArmToLowScaleBack(""));

	m_armToMiddleScaleBack = new ComboButton(m_xOpButton, m_leftOpBumper, true);
	m_armToMiddleScaleBack->WhenPressed(new ArmToMidScaleBack(""));

	m_armToHighScaleBack = new ComboButton(m_yOpButton, m_leftOpBumper, true);
	m_armToHighScaleBack->WhenPressed(new ArmToHighScaleBack(""));


//	left & right trigger: deploy ramps & engage pto
//	left bumper & right trigger: climb
}

Joystick2481* OI::GetDriverStick() {
	return m_driverStick;
}

Joystick2481* OI::GetOperatorStick() {
	return m_operatorStick;
}
