/*
 * DriveTrainDriveLogCommand.h
 *
 *  Created on: Apr 6, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINDRIVELOGCOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINDRIVELOGCOMMAND_H_

#include "CommandBase.h"

class DriveTrainDriveLogCommand : public CommandBase {
private:
	std::ofstream m_stream;

public:
	DriveTrainDriveLogCommand() : CommandBase("DriveTrainDriveLogCommand"){
		Requires(m_driveTrain.get());
	}
	virtual ~DriveTrainDriveLogCommand() {}

	void Initialize() {
		m_stream = std::ofstream("/home/lvuser/Vel.6.csv");
		m_stream<< "time,flDriveVel,frDriveVel,blDriveVel,brDriveVel,flVoltage,frVoltage,blVoltage,brVoltage\n";
	}
	void Execute() {
		m_driveTrain->Drive(0, .6, 0);

		m_stream<< RobotController::GetFPGATime() << "," <<
				m_driveTrain->GetModule(DriveTrain::FRONT_LEFT_MODULE)->GetSpeed() << "," <<
				m_driveTrain->GetModule(DriveTrain::FRONT_RIGHT_MODULE)->GetSpeed() << "," <<
				m_driveTrain->GetModule(DriveTrain::BACK_LEFT_MODULE)->GetSpeed() << "," <<
				m_driveTrain->GetModule(DriveTrain::BACK_RIGHT_MODULE)->GetSpeed() << "," <<

				m_driveTrain->GetModule(DriveTrain::FRONT_LEFT_MODULE)->GetAppliedVoltage() << "," <<
				m_driveTrain->GetModule(DriveTrain::FRONT_RIGHT_MODULE)->GetAppliedVoltage() << "," <<
				m_driveTrain->GetModule(DriveTrain::BACK_LEFT_MODULE)->GetAppliedVoltage() << "," <<
				m_driveTrain->GetModule(DriveTrain::BACK_RIGHT_MODULE)->GetAppliedVoltage() <<

				"\n";

	}
	bool IsFinished() {
		return TimeSinceInitialized() > 56;
	}
	void End() {
		m_driveTrain->Drive(0, 0, 0);
		m_stream.close();
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINDRIVELOGCOMMAND_H_ */
