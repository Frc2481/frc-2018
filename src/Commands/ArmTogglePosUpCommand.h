/*
 * ArmTogglePosUpCommand.h
 *
 *  Created on: Feb 18, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMTOGGLEPOSUPCOMMAND_H_
#define SRC_COMMANDS_ARMTOGGLEPOSUPCOMMAND_H_

#include "CommandBase.h"
#include "RobotParameters.h"

class ArmTogglePosUpCommand : public InstantCommand{
	Command* m_switchFront;
	Command* m_switch2Front;
	Command* m_scaleLowFront;
	Command* m_scaleLow2Front;
	Command* m_scaleMidFront;
	Command* m_scaleMid2Front;
	Command* m_scaleHighFront;
	Command* m_scaleHigh2Front;

	Command* m_switchBack;
	Command* m_switch2Back;
	Command* m_scaleLowBack;
	Command* m_scaleLow2Back;
	Command* m_scaleMidBack;
	Command* m_scaleMid2Back;
	Command* m_scaleHighBack;
	Command* m_scaleHigh2Back;

public:
	ArmTogglePosUpCommand() : InstantCommand("ArmTogglePosUpCommand") {
		m_switchFront = new ArmToSwitchFront("");
		m_switch2Front = new ArmToSwitchFront("");
		m_scaleLowFront = new ArmToLowScaleFront("");
		m_scaleLow2Front = new ArmToLowScale2Front("");
		m_scaleMidFront = new ArmToMidScaleFront("");
		m_scaleMid2Front = new ArmToMidScale2Front("");
		m_scaleHighFront = new ArmToHighScaleFront("");
		m_scaleHigh2Front = new ArmToHighScale2Front("");

		m_switchBack = new ArmToSwitchBack("");
		m_switch2Back = new ArmToSwitchBack("");
		m_scaleLowBack = new ArmToLowScaleBack("");
		m_scaleLow2Back = new ArmToLowScale2Back("");
		m_scaleMidBack = new ArmToMidScaleBack("");
		m_scaleMid2Back = new ArmToMidScale2Back("");
		m_scaleHighBack = new ArmToHighScaleBack("");
		m_scaleHigh2Back = new ArmToHighScale2Back("");
	}
	virtual ~ArmTogglePosUpCommand() {}
	void Initialize() {
		if(CommandBase::m_arm->GetPivotAngle().getDegrees() > 0) {
			printf("got here toggle\n");
			if(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() < RobotParameters::k_switch2FrontAngle) {
				m_switch2Front->Start();
			}
			else if(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() < RobotParameters::k_scaleLowFrontAngle) {
				m_scaleLowFront->Start();
			}
			else if(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() < RobotParameters::k_scaleLow2FrontAngle) {
				m_scaleLow2Front->Start();
			}
			else if(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() <RobotParameters::k_scaleMidFrontAngle) {
				m_scaleMidFront->Start();
			}
			else if(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() < RobotParameters::k_scaleMid2FrontAngle) {
				m_scaleMid2Front->Start();
			}
			else if(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() < RobotParameters::k_scaleHighFrontAngle) {
				m_scaleHighFront->Start();
			}
			else if(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() < RobotParameters::k_scaleHigh2FrontAngle) {
				m_scaleHigh2Front->Start();
			}
		}

		else {
			if(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() < RobotParameters::k_switch2BackAngle) {
				m_switch2Back->Start();
			}
			else if(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() < RobotParameters::k_scaleLowBackAngle) {
				m_scaleLowBack->Start();
			}
			else if(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() < RobotParameters::k_scaleLow2BackAngle) {
				m_scaleLow2Back->Start();
			}
			else if(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() < RobotParameters::k_scaleMidBackAngle) {
				m_scaleMidBack->Start();
			}
			else if(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() < RobotParameters::k_scaleMid2BackAngle) {
				m_scaleMid2Back->Start();
			}
			else if(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() < RobotParameters::k_scaleHighBackAngle) {
				m_scaleHighBack->Start();
			}
			else if(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees() < RobotParameters::k_scaleHigh2BackAngle) {
				m_scaleHigh2Back->Start();
			}
		}
	}
};

#endif /* SRC_COMMANDS_ARMTOGGLEPOSUPCOMMAND_H_ */
