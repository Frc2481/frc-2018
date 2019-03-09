/*
 * AutoCubeAndScaleCommandGroup.h
 *
 *  Created on: Mar 1, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOCUBEANDSCALECOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOCUBEANDSCALECOMMANDGROUP_H_

#include "frc/Commands/CommandGroup.h"
#include "Commands/AutoCubeCommandGroup.h"
#include "Commands/AutoScaleCommandGroup.h"

template<class ARM>
class AutoCubeAndScaleCommandGroup : public CommandGroup{
public:
	AutoCubeAndScaleCommandGroup(std::string cubePath, std::string scalePath, int x, int y) : CommandGroup("AutoCubeAndScaleCommandGroup") {
		AddSequential(new AutoCubeCommandGroup(cubePath, x, y));
		AddSequential(new AutoScaleCommandGroup<ARM>(scalePath));
	}
};

#endif /* SRC_COMMANDS_AUTOCUBEANDSCALECOMMANDGROUP_H_ */
