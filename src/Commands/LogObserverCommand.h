/*
 * LogObserverCommand.h
 *
 *  Created on: Feb 24, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_LOGOBSERVERCOMMAND_H_
#define SRC_COMMANDS_LOGOBSERVERCOMMAND_H_

#include <fstream>
#include <sstream>
#include "CommandBase.h"

class LogObserverCommand : public CommandBase{
private:
	std::ofstream m_stream;
public:
	LogObserverCommand() : CommandBase("LogObserverCommand"){}
	virtual ~LogObserverCommand(){}
	void Initialize() {
		std::stringstream ss;
		ss << "/home/lvuser/ObserverLog_" << DriverStation::GetInstance().GetMatchNumber() << ".csv";
		m_stream = std::ofstream(ss.str());
		m_stream<< "time,x,y,heading\n";
	}

	void Execute() {
		RigidTransform2D pose = CommandBase::m_driveTrain->GetObserver()->GetLastRobotPose();
		m_stream<< RobotController::GetFPGATime() << "," <<
				  pose.getTranslation().getX()<< "," <<
				  pose.getTranslation().getY() << "," <<
				  pose.getRotation().getDegrees() << "\n";
	}
	bool IsFinished() {
		return false;
	}
	void End() {
		m_stream.close();
	}
	void Interrupted() {
		End();
	}
};

#endif /* SRC_COMMANDS_LOGOBSERVERCOMMAND_H_ */
