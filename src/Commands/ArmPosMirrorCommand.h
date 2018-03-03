/*
 * ArmPosMirrorCommand.h
 *
 *  Created on: Feb 19, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMPOSMIRRORCOMMAND_H_
#define SRC_COMMANDS_ARMPOSMIRRORCOMMAND_H_

#include "CommandBase.h"
#include "Subsystems/Arm.h"
#include "ArmBaseCommand.h"

class ArmPosMirrorCommand : public InstantCommand{
private:
	std::map<double, Command*> m_commands;
public:
	ArmPosMirrorCommand() : InstantCommand("ArmPosMirrorCommand"){
		m_commands[ArmToIntakeFront::k_pivotAngle] = new ArmToIntakeBack("");
		m_commands[ArmToIntake2Front::k_pivotAngle] = new ArmToIntake2Back("");
		m_commands[ArmToIntake3Front::k_pivotAngle] = new ArmToIntake3Back("");
		m_commands[ArmToSwitchFront::k_pivotAngle] = new ArmToSwitchBack("");
		m_commands[ArmToLowScaleFront::k_pivotAngle] = new ArmToLowScaleBack("");
		m_commands[ArmToLowScale2Front::k_pivotAngle] = new ArmToLowScale2Back("");
		m_commands[ArmToMidScaleFront::k_pivotAngle] = new ArmToMidScaleBack("");
		m_commands[ArmToMidScale2Front::k_pivotAngle] = new ArmToMidScale2Back("");
		m_commands[ArmToHighScaleFront::k_pivotAngle] = new ArmToHighScaleBack("");
		m_commands[ArmToHighScale2Front::k_pivotAngle] = new ArmToHighScale2Back("");

		m_commands[ArmToIntakeBack::k_pivotAngle] = new ArmToIntakeFront("");
		m_commands[ArmToIntake2Back::k_pivotAngle] = new ArmToIntake2Front("");
		m_commands[ArmToIntake3Back::k_pivotAngle] = new ArmToIntake3Front("");
		m_commands[ArmToSwitchBack::k_pivotAngle] = new ArmToSwitchFront("");
		m_commands[ArmToLowScaleBack::k_pivotAngle] = new ArmToLowScaleFront("");
		m_commands[ArmToLowScale2Back::k_pivotAngle] = new ArmToLowScale2Front("");
		m_commands[ArmToMidScaleBack::k_pivotAngle] = new ArmToMidScaleFront("");
		m_commands[ArmToMidScale2Back::k_pivotAngle] = new ArmToMidScale2Front("");
		m_commands[ArmToHighScaleBack::k_pivotAngle] = new ArmToHighScaleFront("");
		m_commands[ArmToHighScale2Back::k_pivotAngle] = new ArmToHighScale2Front("");
	}
	virtual ~ArmPosMirrorCommand(){}

	void Initialize() {
		auto command = m_commands.find(round(CommandBase::m_arm->GetDesiredPivotAngle().getDegrees()));
		if (command != m_commands.end()) {
			command->second->Start();
		}
	}
};

#endif /* SRC_COMMANDS_ARMPOSMIRRORCOMMAND_H_ */
