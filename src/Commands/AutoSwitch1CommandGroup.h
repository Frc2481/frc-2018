/*
 * AutoSwitch1CommandGroup.h
 *
 *  Created on: Feb 8, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOSWITCH1COMMANDGROUP_H_
#define SRC_COMMANDS_AUTOSWITCH1COMMANDGROUP_H_

#include "CommandBase.h"

class AutoSwitch1CommandGroup : public CommandGroup{
public:
	AutoSwitch1CommandGroup(std::string pathToSwitch, std::string pathToMeeting) : CommandGroup("AutoSwitch1CommandGroup"){
		AddSequential(new DriveTrainFollowPath(pathToSwitch));
//		AddSequential(new ArmToSwitchFront("")); //front or back?
		AddSequential(new IntakeReleaseCubeCommandGroup(1.0));
	}
};

#endif /* SRC_COMMANDS_AUTOSWITCH1COMMANDGROUP_H_ */
