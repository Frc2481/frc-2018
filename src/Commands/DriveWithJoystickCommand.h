/*
 * DriveWithJoystickCommand.h
 *
 *  Created on: Nov 6, 2017
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_2017COMMANDS_DRIVEWITHJOYSTICKCOMMAND_H_
#define SRC_COMMANDS_2017COMMANDS_DRIVEWITHJOYSTICKCOMMAND_H_

#include "CommandBase.h"
#include "../XboxController.h"
#include "Components/Joystick2481.h"

class DriveWithJoystickCommand : public CommandBase{
public:
	DriveWithJoystickCommand() : CommandBase("DriveWJoystick") {
		Requires(m_driveTrain.get());
	}
	virtual ~DriveWithJoystickCommand() {

	}

	void Initialize() {}

	void Execute() {
		m_driveTrain->Drive(
				oi->GetDriverStick()->GetRawAxis(XB_LEFT_X_AXIS),
				-oi->GetDriverStick()->GetRawAxis(XB_LEFT_Y_AXIS),
				-oi->GetDriverStick()->GetRawAxisTwist(XB_RIGHT_X_AXIS));
	}

	void End() {}

	bool IsFinished(){
		return false;
	}

	void Interrupted() {
		m_driveTrain->Stop();
	}
};

#endif /* SRC_COMMANDS_2017COMMANDS_DRIVEWITHJOYSTICKCOMMAND_H_ */
