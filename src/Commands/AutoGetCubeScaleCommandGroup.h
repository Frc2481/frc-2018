/*
 * AutoGetCubeScaleCommandGroup.h
 *
 *  Created on: Feb 24, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOGETCUBESCALECOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOGETCUBESCALECOMMANDGROUP_H_

#include "Commands/CommandGroup.h"

class AutoGetCubeScaleCommandGroup : public CommandGroup{
public:
	AutoGetCubeScaleCommandGroup(std::string cubePath, std::string scalePath) : CommandGroup("AutoGetCubeScaleCommandGroup"){
		AddSequential(new ObserverResetPosCommand());
		AddSequential(new AutoCubeCommandGroup(cubePath, -1, -1));
		AddSequential(new AutoScaleCommandGroup(scalePath));
	}
};

#endif /* SRC_COMMANDS_AUTOGETCUBESCALECOMMANDGROUP_H_ */
