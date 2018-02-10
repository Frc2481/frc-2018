/*
 * AutoGetCubeCommandGroup.h
 *
 *  Created on: Feb 8, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOGETCUBECOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOGETCUBECOMMANDGROUP_H_

#include "CommandBase.h"
#include "Subsystems/Arm.h"

class AutoGetCubeCommandGroup : public CommandGroup{
public:
	AutoGetCubeCommandGroup(const std::string name) : CommandGroup("AutoGetCubeCommandGroup") {
		//path to inner cubes
		AddSequential(new ArmToIntakeFront());
		AddSequential(new IntakeAcquireCubeCommandGroup());
		AddSequential(new ArmToStow());
	}
};

#endif /* SRC_COMMANDS_AUTOGETCUBECOMMANDGROUP_H_ */
