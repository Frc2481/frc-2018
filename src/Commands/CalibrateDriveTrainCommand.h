/*
 * CalibrateDriveTrainCommand.h
 *
 *  Created on: Jan 13, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_CALIBRATEDRIVETRAINCOMMAND_H_
#define SRC_COMMANDS_CALIBRATEDRIVETRAINCOMMAND_H_

#include <Subsystems/DriveTrain.h>
#include <Components/SwerveModule.h>
#include "../CommandBase.h"
#include <Components/CTREMagEncoder.h>

class CalibrateDriveTrainCommand : public InstantCommand{
public:
	CalibrateDriveTrainCommand() : InstantCommand("CalibrateDriveTrainCommand") {
		SetRunWhenDisabled(true);
	}
	virtual ~CalibrateDriveTrainCommand() {}

	void Initialize() {
		CommandBase::m_driveTrain->GetModule(DriveTrain::SwerveModuleType::FRONT_LEFT_MODULE)->GetSteerEncoder()->Calibrate();
		CommandBase::m_driveTrain->GetModule(DriveTrain::SwerveModuleType::FRONT_RIGHT_MODULE)->GetSteerEncoder()->Calibrate();
		CommandBase::m_driveTrain->GetModule(DriveTrain::SwerveModuleType::BACK_LEFT_MODULE)->GetSteerEncoder()->Calibrate();
		CommandBase::m_driveTrain->GetModule(DriveTrain::SwerveModuleType::BACK_RIGHT_MODULE)->GetSteerEncoder()->Calibrate();
	}
};

#endif /* SRC_COMMANDS_CALIBRATEDRIVETRAINCOMMAND_H_ */
