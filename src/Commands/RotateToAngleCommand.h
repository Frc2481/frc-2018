/*
 * RotateToAngleCommandV2.h
 *
 *  Created on: Nov 6, 2017
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_V2_2017COMMANDS_ROTATETOANGLECOMMANDV2_H_
#define SRC_COMMANDS_V2_2017COMMANDS_ROTATETOANGLECOMMANDV2_H_

#include "CommandBase.h"

class RotateToAngleCommandV2 : public CommandBase{
public:
	RotateToAngleCommandV2();
	virtual ~RotateToAngleCommandV2();

	void Initialize(){
		m_driveTrain->GetModule(DriveTrain2017::FRONT_LEFT_MODULE)->SetAngle(Rotation2D::fromDegrees(45));
		m_driveTrain->GetModule(DriveTrain2017::FRONT_RIGHT_MODULE)->SetAngle(Rotation2D::fromDegrees(-45));
		m_driveTrain->GetModule(DriveTrain2017::BACK_LEFT_MODULE)->SetAngle(Rotation2D::fromDegrees(-135));
		m_driveTrain->GetModule(DriveTrain2017::BACK_RIGHT_MODULE)->SetAngle(Rotation2D::fromDegrees(135));
	}
	void Execute(){

	}
	bool IsFinished(){

	}
	void End(){

	}
	void Interrupted(){

	}
};

#endif /* SRC_COMMANDS_V2_2017COMMANDS_ROTATETOANGLECOMMANDV2_H_ */
