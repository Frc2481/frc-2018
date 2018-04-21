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
private:
	double m_prevY;
	bool m_endFlag;

public:
	DriveTrainSinusoidalJitter() : CommandBase("DriveTrainSinusoidalJitter"){
		m_prevY = 0.0;
		m_endFlag = false;
	}
	virtual ~DriveTrainSinusoidalJitter() {}

	void Initialize() {
		m_prevY = 0.0;
		m_endFlag = false;
	}

	void Execute() {
		double period = 2 * M_PI;
		double dy = period * cos(TimeSinceInitialized() * period);
		double y = sin(TimeSinceInitialized() * period);
		y *= (y > 0 ? 0.5 : 1.0);
		m_driveTrain->Drive(0.0, y, 0.0);

		if((y <= -0.3 && dy < 0) || (dy > 0 && y < -0.6)) {
			m_endFlag = true;
		}
		else {
			m_endFlag = false;
		}

		m_prevY = y;
	}

	bool IsFinished() {
		return m_intake->HasCube() && m_endFlag;
	}

	void End() {
		m_driveTrain->Stop();
	}

	void Interrupted() {
		End();
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINSINUSOIDALJITTER_H_ */
