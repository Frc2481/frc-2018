/*
 * DriveTrainDriveCommand.h
 *
 *  Created on: Feb 27, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_DRIVETRAINDRIVECOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINDRIVECOMMAND_H_

#include "CommandBase.h"
#include "Subsystems/DriveTrain.h"
#include "Commands/TimedCommand.h"

class DriveTrainDriveCommand : public TimedCommand{
private:
	double m_x;
	double m_y;
	double m_theta;
public:
	DriveTrainDriveCommand(double x, double y, double theta, double timeout) : TimedCommand("DriveTrainDriveCommand", timeout) {
		m_x = x;
		m_y = y;
		m_theta = theta;
		Requires(CommandBase::m_driveTrain.get());
	}
	virtual ~DriveTrainDriveCommand(){}

	void Initialize() {
		CommandBase::m_driveTrain->Drive(m_x, m_y, m_theta);
	}

	void End() {
		CommandBase::m_driveTrain->Stop();
	}

	void Interrupted() {
		End();
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINDRIVECOMMAND_H_ */
