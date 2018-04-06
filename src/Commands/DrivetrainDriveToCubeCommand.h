/*
 * DrivetrainDriveToCubeCommand.h
 *
 *  Created on: Apr 5, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINDRIVETOCUBECOMMAND_H_
#define SRC_COMMANDS_DRIVETRAINDRIVETOCUBECOMMAND_H_

#include "CommandBase.h"
#include "Limelight.h"

class DrivetrainDriveToCubeCommand : public CommandBase{
private:
	DriveController* m_driveController;
	LimeLight* m_limelight;
public:
	DrivetrainDriveToCubeCommand() : CommandBase("DrivetrainDriveToCubeCommand") {
		m_driveController = m_driveTrain->GetDriveController();
	}
	virtual ~DrivetrainDriveToCubeCommand() {}

	void Initialize() {
		m_driveController->EnableController();
		m_limelight->ActivatePowerCubePipeline();
	}

	void Execute() {
		if(!(m_limeLight->getPowerCubeTargetValid())) {
			End();
		}

		RigidTransform2D targetPos = m_limeLight->GetPowerCubePose();

		m_driveController->SetFieldTarget(targetPos);

		RigidTransform2D driveSignal = m_driveController->GetDriveControlSignal();
		m_driveTrain->Drive(driveSignal.getTranslation().getX(),
							driveSignal.getTranslation().getY(),
							driveSignal.getRotation().getDegrees());
		targetPos = 0;

	}
	void Interrupted() {
		End();
	}

	bool IsFinished() {

	}

	void End() {
		m_driveTrain->Drive(0, 0, 0);
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINDRIVETOCUBECOMMAND_H_ */
