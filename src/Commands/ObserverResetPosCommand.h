/*
 * ObserverResetPosCommand.h
 *
 *  Created on: Jan 19, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_OBSERVERRESETPOSCOMMAND_H_
#define SRC_COMMANDS_OBSERVERRESETPOSCOMMAND_H_

#include "../CommandBase.h"

class ObserverResetPosCommand : public InstantCommand {
private:
	RigidTransform2D m_pose;
public:
	ObserverResetPosCommand(RigidTransform2D pose = RigidTransform2D()) : InstantCommand("ObserverResetPosCommand") {
		SetRunWhenDisabled(true);
		m_pose = pose;
	}

	void Initialize() {
		CommandBase::m_driveTrain->ZeroGyro();
		CommandBase::m_driveTrain->ResetRobotPose(m_pose);
	}
};

#endif /* SRC_COMMANDS_OBSERVERRESETPOSCOMMAND_H_ */
