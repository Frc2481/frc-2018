/*
 * ArmTogglePosDownCommand.h
 *
 *  Created on: Feb 18, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMTOGGLEPOSDOWNCOMMAND_H_
#define SRC_COMMANDS_ARMTOGGLEPOSDOWNCOMMAND_H_

#include "CommandBase.h"
#include "RobotParameters.h"

class ArmTogglePosDownCommand : public frc::InstantCommand{
private:
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
	ArmTogglePosDownCommand() : InstantCommand("ArmTogglePosDownCommand") {
		m_switchFront = new ArmToSwitchFront("");
		m_switch2Front = new ArmToSwitch2Front("");
		m_scaleLowFront = new ArmToLowScaleFront("");
		m_scaleLow2Front = new ArmToLowScale2Front("");
		m_scaleMidFront = new ArmToMidScaleFront("");
		m_scaleMid2Front = new ArmToMidScale2Front("");
		m_scaleHighFront = new ArmToHighScaleFront("");
		m_scaleHigh2Front = new ArmToHighScale2Front("");

		m_switchBack = new ArmToSwitchBack("");
		m_switch2Back = new ArmToSwitch2Back("");
		m_scaleLowBack = new ArmToLowScaleBack("");
		m_scaleLow2Back = new ArmToLowScale2Back("");
		m_scaleMidBack = new ArmToMidScaleBack("");
		m_scaleMid2Back = new ArmToMidScale2Back("");
		m_scaleHighBack = new ArmToHighScaleBack("");
		m_scaleHigh2Back = new ArmToHighScale2Back("");
	}
	virtual ~ArmTogglePosDownCommand() {}
	void Initialize() {
		double pivotAngle = CommandBase::m_arm->GetDesiredPivotAngle().getDegrees();
		if(pivotAngle > 0) {
			pivotAngle += .001; //ensure doubles compare correctly
			if(pivotAngle < ArmToHighScale2Front::k_pivotAngle) {
				m_scaleHigh2Front->Start();
			}
			else if(pivotAngle < ArmToHighScaleFront::k_pivotAngle) {
				m_scaleHighFront->Start();
			}
			else if(pivotAngle < ArmToMidScale2Front::k_pivotAngle) {
				m_scaleMid2Front->Start();
			}
			else if(pivotAngle < ArmToMidScaleFront::k_pivotAngle) {
				m_scaleMidFront->Start();
			}
			else if(pivotAngle < ArmToLowScale2Front::k_pivotAngle) {
				m_scaleLow2Front->Start();
			}
			else if(pivotAngle <  ArmToLowScaleFront::k_pivotAngle) {
				m_scaleLowFront->Start();
			}
			else if(pivotAngle <  ArmToSwitch2Front::k_pivotAngle) {
				m_switch2Front->Start();
			}
			else if(pivotAngle <  ArmToSwitchFront::k_pivotAngle) {
				m_switchFront->Start();
			}
		}

		else {
			pivotAngle -= .001; //ensure doubles compare correctly
			if(pivotAngle > ArmToHighScale2Back::k_pivotAngle) {
				m_scaleHigh2Back->Start();
			}
			else if(pivotAngle > ArmToHighScaleBack::k_pivotAngle) {
				m_scaleHighBack->Start();
			}
			else if(pivotAngle > ArmToMidScale2Back::k_pivotAngle) {
				m_scaleMid2Back->Start();
			}
			else if(pivotAngle > ArmToMidScaleBack::k_pivotAngle) {
				m_scaleMidBack->Start();
			}
			else if(pivotAngle > ArmToLowScale2Back::k_pivotAngle) {
				m_scaleLow2Back->Start();
			}
			else if(pivotAngle > ArmToLowScaleBack::k_pivotAngle) {
				m_scaleLowBack->Start();
			}
			else if(pivotAngle > ArmToSwitch2Back::k_pivotAngle) {
				m_switch2Back->Start();
			}
			else if(pivotAngle > ArmToSwitchBack::k_pivotAngle) {
				m_switchBack->Start();
			}
		}
	}
};

#endif /* SRC_COMMANDS_ARMTOGGLEPOSDOWNCOMMAND_H_ */
