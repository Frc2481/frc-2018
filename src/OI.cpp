
#include <Commands/ArmExtendCommand.h>
#include <Commands/ArmRetractCommand.h>
#include <Commands/IntakeAcquireCubeCommandGroup.h>
#include <commands/IntakeGrabCubeCommandGroup.h>
#include <Commands/ArmIntakeFrontPosToggleCommand.h>
#include <Commands/ArmIntakeBackPosToggleCommand.h>
#include <Commands/ArmPivotDownCommand.h>
#include <Commands/ArmPivotUpCommand.h>
#include <Commands/ArmTogglePosUpCommand.h>
#include <Commands/ArmTogglePosDownCommand.h>
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
#include "Subsystems/Arm.h"
#include "Commands/ArmPosMirrorCommand.h"
#include "Commands/DriveTrainSetFieldCentricCommand.h"
#include "Commands/IntakeControlReleaseSpeedCubeCommand.h"
#include "Commands/IntakeWhileHeldCommand.h"

#include "Commands/DriveTrainNearWinchCommand.h"
#include "Commands/DriveTrainFarWinchCommand.h"
#include "Commands/ArmToIntakePos3Command.h"
#include "Commands/DriveTrainZeroGyroCommand.h"

OI::OI() {
	// Process operator interface input here.
	m_driverStick = new Joystick2481(0);
	m_operatorStick = new Joystick2481(1);
//	m_climbStick = new Joystick2481(2);

//driver
//	m_aDriverButton = new JoystickButton(m_driverStick, XB_A_BUTTON);
//	m_bDriverButton = new JoystickButton(m_driverStick, XB_B_BUTTON);
//	m_xDriverButton = new JoystickButton(m_driverStick, XB_X_BUTTON);
//	m_yDriverButton = new JoystickButton(m_driverStick, XB_Y_BUTTON);
//
//	m_leftDriverBumper = new JoystickButton(m_driverStick, XB_LEFT_BUMPER);
//	m_rightDriverBumper = new JoystickButton(m_driverStick, XB_RIGHT_BUMPER);

//operator
	m_aOpButton = new JoystickButton(m_operatorStick, XB_A_BUTTON);
	m_bOpButton = new JoystickButton(m_operatorStick, XB_B_BUTTON);
	m_xOpButton = new JoystickButton(m_operatorStick, XB_X_BUTTON);
	m_yOpButton = new JoystickButton(m_operatorStick, XB_Y_BUTTON);

	m_leftOpBumper = new JoystickButton(m_operatorStick, XB_LEFT_BUMPER);
	m_rightOpBumper = new JoystickButton(m_operatorStick, XB_RIGHT_BUMPER);

	m_startButton = new JoystickButton(m_operatorStick, XB_START_BUTTON);
	m_backButton = new JoystickButton(m_operatorStick, XB_BACK_BUTTON);

	m_leftTrigger = new AnalogJoystickButton(m_operatorStick, XB_LEFT_TRIGGER, 0.5);
	m_rightTrigger = new AnalogJoystickButton(m_operatorStick, XB_RIGHT_TRIGGER, 0.5);


//driver
	m_shifterButton = new JoystickButton(m_driverStick, XB_RIGHT_BUMPER);
	m_shifterButton->WhileHeld(new DriveTrainShiftCommand());

	m_fieldCentricButton = new JoystickButton(m_driverStick, XB_LEFT_BUMPER);
	m_fieldCentricButton->WhenPressed(new DriveTrainSetFieldCentricCommand(true));
	m_fieldCentricButton->WhenReleased(new DriveTrainSetFieldCentricCommand(false));

	m_intakeCubeButton = new AnalogJoystickButton(m_driverStick, XB_RIGHT_TRIGGER, 0.5);
	m_intakeCubeButton->WhileHeld(new IntakeWhileHeldCommand());

//	m_climberButton = new AnalogJoystickButton(m_driverStick, XB_LEFT_TRIGGER, 0.5);
//	m_climberButton->ToggleWhenPressed(new ClimberExtendCommand()); //figure out to toggle
//	m_climberButton->ToggleWhenPressed(new CLimberRetractCommand()); //figure out to toggle

	m_armToIntakeFront = new JoystickButton(m_driverStick, XB_A_BUTTON);
	m_armToIntakeFront->WhenPressed(new ArmIntakeFrontPosToggleCommand());

	m_armToIntakeBack = new JoystickButton(m_driverStick, XB_X_BUTTON);
	m_armToIntakeBack->WhenPressed(new ArmIntakeBackPosToggleCommand());

	m_stowButtonDriver = new JoystickButton(m_driverStick, XB_Y_BUTTON);
	m_stowButtonDriver->WhenPressed(new ArmToStow(""));

//	m_releaseSlowCubeButton = new JoystickButton(m_driverStick, XB_B_BUTTON);
//	m_releaseSlowCubeButton->WhileHeld(new IntakeReleaseCubeCommandGroup(0.2));
//	m_releaseSlowCubeButton->WhenReleased(new IntakeRollerOffCommand());

//	m_releaseFastCubeButton = new JoystickButton(m_driverStick, XB_Y_BUTTON);
//	m_releaseFastCubeButton->WhileHeld(new IntakeReleaseCubeCommandGroup(1));
//	m_releaseFastCubeButton->WhenReleased(new IntakeRollerOffCommand());

	m_intake3PosButton = new JoystickButton(m_driverStick, XB_B_BUTTON);
	m_intake3PosButton->WhenPressed(new ArmToIntakePos3Command());

	m_driveTrainZeroGyro = new JoystickButton(m_driverStick, XB_START_BUTTON);
	m_driveTrainZeroGyro->WhenPressed(new DriveTrainZeroGyroCommand());


//operator

	m_extenderButton = new AnalogJoystickButton(m_operatorStick, XB_LEFT_Y_AXIS, 0.25); //left stick up
	m_extenderButton->WhileHeld(new ArmExtendCommand());

	m_retractButton = new AnalogJoystickButton(m_operatorStick, XB_LEFT_Y_AXIS, -0.25); //left stick down
	m_retractButton->WhileHeld(new ArmRetractCommand());

	m_pivotUpButton = new AnalogJoystickButton(m_operatorStick, XB_RIGHT_Y_AXIS, -0.25); // right stick up
	m_pivotUpButton->WhileHeld(new ArmPivotUpCommand());

	m_pivotDownButton = new AnalogJoystickButton(m_operatorStick, XB_RIGHT_Y_AXIS, 0.25); //right stick down
	m_pivotDownButton->WhileHeld(new ArmPivotDownCommand());

//arm positions
//front
	m_armToSwitchFront = new ComboButton(m_aOpButton, m_rightTrigger, false);
	m_armToSwitchFront->WhenPressed(new ArmToSwitchFront(""));

	m_armToLowScaleFront = new ComboButton(m_bOpButton, m_rightTrigger, false);
	m_armToLowScaleFront->WhenPressed(new ArmToLowScaleFront(""));

	m_armToMiddleScaleFront = new ComboButton(m_xOpButton, m_rightTrigger, false);
	m_armToMiddleScaleFront->WhenPressed(new ArmToMidScaleFront(""));

	m_armToHighScaleFront = new ComboButton(m_yOpButton, m_rightTrigger, false);
	m_armToHighScaleFront->WhenPressed(new ArmToHighScaleFront(""));

//back
	m_armToSwitchBack = new ComboButton(m_aOpButton, m_rightTrigger, true);
	m_armToSwitchBack->WhenPressed(new ArmToSwitchBack(""));

	m_armToLowScaleBack = new ComboButton(m_bOpButton, m_rightTrigger, true);
	m_armToLowScaleBack->WhenPressed(new ArmToLowScaleBack(""));

	m_armToMiddleScaleBack = new ComboButton(m_xOpButton, m_rightTrigger, true);
	m_armToMiddleScaleBack->WhenPressed(new ArmToMidScaleBack(""));

	m_armToHighScaleBack = new ComboButton(m_yOpButton, m_rightTrigger, true);
	m_armToHighScaleBack->WhenPressed(new ArmToHighScaleBack(""));

	m_exchangeFrontButton = new ComboButton(m_startButton, m_rightTrigger, false);
	m_exchangeFrontButton->WhenPressed(new ArmToExchangeFront(""));

	m_exchangeBackButton = new ComboButton(m_startButton, m_rightTrigger, true);
	m_exchangeBackButton->WhenPressed(new ArmToExchangeBack(""));

	m_variableReleaseSpeed = new AnalogJoystickButton(m_operatorStick, XB_LEFT_TRIGGER, 0.1);
	m_variableReleaseSpeed->WhileHeld(new IntakeControlReleaseSpeedCubeCommand());
//	m_intakeButton = new AnalogJoystickButton(m_operatorStick, XB_RIGHT_TRIGGER, 0.5);
//	m_intakeButton->WhileHeld(new IntakeGrabCubeCommandGroup());
//	m_intakeButton->WhenReleased(new IntakeStopCubeCommandGroup());

	m_nextPos = new POVJoystickButton(m_operatorStick, 0, XB_DPAD_TOP);
	m_nextPos->WhenPressed(new ArmTogglePosUpCommand());

	m_lastPos = new POVJoystickButton(m_operatorStick, 0, XB_DPAD_BOTTOM);
	m_lastPos->WhenPressed(new ArmTogglePosDownCommand());

	m_mirrorArmPos = new JoystickButton(m_operatorStick, XB_RIGHT_BUMPER);
	m_mirrorArmPos->WhenPressed(new ArmPosMirrorCommand());

	m_stowButtonOp = new JoystickButton(m_operatorStick, XB_LEFT_BUMPER);
	m_stowButtonOp->WhenPressed(new ArmToStow(""));

	//intake release on left trigger?

//climb controller
//	m_innerWinch = new JoystickButton(m_climbStick, XB_A_BUTTON);
//	m_innerWinch->WhileHeld(new DriveTrainNearWinchCommand());
//
//	m_outerWinch = new JoystickButton(m_climbStick, XB_B_BUTTON);
//	m_outerWinch->WhileHeld(new DriveTrainFarWinchCommand());

//	m_extensionOpenLoop = new JoystickButton(m_climbStick, XB_RIGHT_BUMPER);
//	m_extensionOpenLoop->WhileHeld(new ArmExtendCommand());
}

Joystick2481* OI::GetDriverStick() {
	return m_driverStick;
}

Joystick2481* OI::GetOperatorStick() {
	return m_operatorStick;
}
