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
#include "Components/SwerveModule.h"
#include "AHRS.h"

class LogObserverCommand : public CommandBase{
private:
	std::ofstream m_stream;
public:
	LogObserverCommand() : CommandBase("LogObserverCommand"){
		SetRunWhenDisabled(true);
	}
	virtual ~LogObserverCommand(){}
	void Initialize() {
		std::stringstream ss;
		ss << "/home/lvuser/ObserverLog_" << DriverStation::GetInstance().GetMatchNumber() << ".csv";
		m_stream = std::ofstream(ss.str());
		m_stream<< "time,x,y,heading, flSteer, frSteer, blSteer, brSteer, flDrive, frDrive, blDrive, brDrive, rawGyro, rawAccelX, rawAccelY, yaw, pigeon yaw, pigeon raw\n";
	}

	void Execute() {
		double ypr[3];
		m_driveTrain->GetPigeonImu()->GetYawPitchRoll(ypr);
		RigidTransform2D pose = CommandBase::m_driveTrain->GetObserver()->GetLastRobotPose();
		m_stream<< RobotController::GetFPGATime() << "," <<
				  pose.getTranslation().getX()<< "," <<
				  pose.getTranslation().getY() << "," <<
				  pose.getRotation().getDegrees() <<  "," <<
				  m_driveTrain->GetModule(DriveTrain::FRONT_LEFT_MODULE)->GetAngle().getDegrees() << "," <<
				  m_driveTrain->GetModule(DriveTrain::FRONT_RIGHT_MODULE)->GetAngle().getDegrees() << "," <<
				  m_driveTrain->GetModule(DriveTrain::BACK_LEFT_MODULE)->GetAngle().getDegrees() << "," <<
				  m_driveTrain->GetModule(DriveTrain::BACK_RIGHT_MODULE)->GetAngle().getDegrees() << "," <<
				  m_driveTrain->GetModule(DriveTrain::FRONT_LEFT_MODULE)->GetDistance().getX() << "," <<
				  m_driveTrain->GetModule(DriveTrain::FRONT_RIGHT_MODULE)->GetDistance().getX() << "," <<
				  m_driveTrain->GetModule(DriveTrain::BACK_LEFT_MODULE)->GetDistance().getX() << "," <<
				  m_driveTrain->GetModule(DriveTrain::BACK_RIGHT_MODULE)->GetDistance().getX() << "," <<
				  m_driveTrain->GetImu()->GetRawGyroZ() << "," <<
				  m_driveTrain->GetImu()->GetRawAccelX() << "," <<
				  m_driveTrain->GetImu()->GetRawAccelY() << "," <<
				  m_driveTrain->GetImu()->GetYaw() << "," <<
				  m_driveTrain->GetPigeonImu()->GetFusedHeading() << "," <<
				  ypr[0] << "\n";
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
