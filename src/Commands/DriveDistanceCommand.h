/*
 * DriveDistanceCommand.h
 *
 *  Created on: Nov 13, 2017
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVEDISTANCECOMMAND_H_
#define SRC_COMMANDS_DRIVEDISTANCECOMMAND_H_

#include "CommandBase.h"

class DriveDistanceCommand : public CommandBase{
private:
	Translation2D m_distance;
public:
	DriveDistanceCommand(Translation2D &distance) {
		m_distance = distance;
		Requires(m_driveTrain.get());
	}
	virtual ~DriveDistanceCommand();

	void Initialize() {
		m_driveTrain->DriveCloseLoopDistance(m_distance);
	}

	void Execute() {

	}

	void Interrupted() {

	}

	bool IsFinished() {
		m_driveTrain->IsSteerOnTarget()
	}

	void End() {

	}

};


#endif /* SRC_COMMANDS_DRIVEDISTANCECOMMAND_H_ */
