/*
 * ReloadNewPathsCommand.h
 *
 *  Created on: Feb 23, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_RELOADNEWPATHSCOMMAND_H_
#define SRC_COMMANDS_RELOADNEWPATHSCOMMAND_H_

#include "CommandBase.h"

class ReloadNewPathsCommand : public frc::InstantCommand {
public:
	ReloadNewPathsCommand() : InstantCommand("ReloadNewPathsCommand") {
		SetRunWhenDisabled(true);
	}
	virtual ~ReloadNewPathsCommand(){}

	void Initialize() {
		CommandBase::m_pathManager->ReloadPaths();
	}
};

#endif /* SRC_COMMANDS_RELOADNEWPATHSCOMMAND_H_ */
