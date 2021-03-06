/*
 * ArmClearStickyFaults.h
 *
 *  Created on: Feb 22, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_ARMCLEARSTICKYFAULTS_H_
#define SRC_COMMANDS_ARMCLEARSTICKYFAULTS_H_

#include "CommandBase.h"

class ArmClearStickyFaults : public frc::InstantCommand{
public:
	ArmClearStickyFaults() : InstantCommand("ArmClearStickyFaults"){
		SetRunWhenDisabled(true);
	}
	virtual ~ArmClearStickyFaults(){}

	void Initialize() {
		CommandBase::m_arm->ClearStickyFaults();
	}
};

#endif /* SRC_COMMANDS_ARMCLEARSTICKYFAULTS_H_ */
