/*
 * DriveTrainRunDriveTestCommand.h
 *
 *  Created on: Jan 15, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DIAG_DRIVETRAINRUNDRIVETESTCOMMAND_H_
#define SRC_COMMANDS_DIAG_DRIVETRAINRUNDRIVETESTCOMMAND_H_

#include "CommandBase.h"

class DriveTrainRunDriveTestCommand : public CommandBase {
private:
	double m_x;
	bool m_countUp;

public:
	DriveTrainRunDriveTestCommand() : CommandBase("DriveTrainRunDriveTest") {
		Requires(m_driveTrain.get());
		m_x = -1;
		m_countUp = true;
	}

	void Execute() {
		m_driveTrain->Drive(m_x, 1, 0);
		if(m_countUp == true) {
			m_x += 0.1;
		} else {
			m_x -= 0.1;
		}
		if(m_x > 1) {
			m_countUp = false;
		} else if(m_x < -1) {
			m_countUp = true;
		}

	}

	bool IsFinished() {
		return false;
	}

	void Interrupted() {
		m_driveTrain->Drive(0, 0, 0);
	}
};

#endif /* SRC_COMMANDS_DIAG_DRIVETRAINRUNDRIVETESTCOMMAND_H_ */
