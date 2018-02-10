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
public:
	ObserverResetPosCommand() : InstantCommand("ObserverResetPosCommand") {
		SetRunWhenDisabled(true);
	}

	void Initialize() {
		CommandBase::m_driveTrain->ResetRobotPose();
	}
};

#endif /* SRC_COMMANDS_OBSERVERRESETPOSCOMMAND_H_ */
