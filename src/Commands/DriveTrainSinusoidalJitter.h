/*
 * DriveTrainSinusoidalJitter.h
 *
 *  Created on: Apr 20, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINSINUSOIDALJITTER_H_
#define SRC_COMMANDS_DRIVETRAINSINUSOIDALJITTER_H_

#include "CommandBase.h"

class DriveTrainSinusoidalJitter : public CommandBase {
public:
	DriveTrainSinusoidalJitter() : CommandBase("DriveTrainSinusoidalJitter"){

	}
	virtual ~DriveTrainSinusoidalJitter() {}

	void Initialize() {

	}
	void Execute() {
		double y = sin(TimeSinceInitialized() * 2 * M_PI);
		y *= (y > 0 ? 0.5 : 1.0);
		m_driveTrain->Drive(0.0, y, 0.0);
	}

	bool IsFinished() {
		return m_intake->HasCube();
	}

	void End() {
		m_driveTrain->Stop();
	}

	void Interrupted() {
		End();
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINSINUSOIDALJITTER_H_ */
