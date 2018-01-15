
#include "OI.h"
#include <WPILib.h>
#include "Components/Joystick2481.h"
#include "Commands/DriveTrainShiftCommand.h"
#include "XboxController.h"

OI::OI() {
	// Process operator interface input here.
	m_driverStick = new Joystick2481(0);
	m_operatorStick = new Joystick2481(1);

	m_shifterButton = new JoystickButton(m_driverStick, XB_RIGHT_BUMPER);
	m_shifterButton->WhileHeld(new DriveTrainShiftCommand());
}

Joystick2481* OI::GetDriverStick() {
	return m_driverStick;
}

Joystick2481* OI::GetOperatorStick() {
	return m_operatorStick;
}
