/*
 * AutoCubeAndSwitchCommandGroup.h
 *
 *  Created on: Mar 1, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOCUBEANDSWITCHCOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOCUBEANDSWITCHCOMMANDGROUP_H_

#include "frc/Commands/CommandGroup.h"
#include "Commands/AutoCubeCommandGroup.h"
#include "Commands/AutoSwitchCommandGroup.h"

class AutoCubeAndSwitchCommandGroup : public CommandGroup{
public:
	AutoCubeAndSwitchCommandGroup(std::string cubePath, std::string switchPath, int xCube, int yCube, int xSwitch, int ySwitch) : CommandGroup("AutoCubeAndSwitchCommandGroup") {
		AddSequential(new AutoCubeCommandGroup(cubePath, xCube, yCube));
		AddSequential(new AutoSwitchCommandGroup(switchPath, false, xSwitch, ySwitch));
	}
};

#endif /* SRC_COMMANDS_AUTOCUBEANDSWITCHCOMMANDGROUP_H_ */
