/*
 * AutoGetCubeSwitchCommandGroup.h
 *
 *  Created on: Feb 24, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOGETCUBESWITCHCOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOGETCUBESWITCHCOMMANDGROUP_H_

#include "Commands/CommandGroup.h"

class AutoGetCubeSwitchCommandGroup : public CommandGroup{
public:
	AutoGetCubeSwitchCommandGroup(std::string cubePath, std::string switchPath, bool isFrontIntakeRelease) : CommandGroup("AutoGetCubeScaleCommandGroup"){
		AddSequential(new ObserverResetPosCommand());
		AddSequential(new AutoCubeCommandGroup(cubePath, -1, -1));
		AddSequential(new AutoSwitchCommandGroup(switchPath, isFrontIntakeRelease));
	}
};

#endif /* SRC_COMMANDS_AUTOGETCUBESWITCHCOMMANDGROUP_H_ */
