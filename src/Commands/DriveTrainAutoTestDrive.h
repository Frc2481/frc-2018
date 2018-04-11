/*
 * DriveTrainAutoTestDrive.h
 *
 *  Created on: Apr 2, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINAUTOTESTDRIVE_H_
#define SRC_COMMANDS_DRIVETRAINAUTOTESTDRIVE_H_

#include "CommandBase.h"

class DriveTrainAutoTestDrive : public CommandBase{
private:
	std::ofstream m_stream;

public:
	DriveTrainAutoTestDrive() : CommandBase("DriveTrainAutoTestDrive"){
		Requires(m_driveTrain.get());
	}
	virtual ~DriveTrainAutoTestDrive() {}

	void Initialize() {
		m_stream = std::ofstream("/home/lvuser/RampDrive.csv");
		m_stream<< "time,flDriveVel,frDriveVel,blDriveVel,brDriveVel,flVoltage,frVoltage,blVoltage,brVoltage\n";
	}
	void Execute() {
		m_driveTrain->Drive(0, TimeSinceInitialized() * 1 / 48.0, 0);

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

#endif /* SRC_COMMANDS_DRIVETRAINAUTOTESTDRIVE_H_ */
