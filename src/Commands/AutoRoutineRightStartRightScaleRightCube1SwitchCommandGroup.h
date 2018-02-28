/*
 * AutoRoutineRightStartRightScaleRightCube1RightSwitchCommandGroup.h
 *
 *  Created on: Feb 20, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_AUTOROUTINERIGHTSTARTRIGHTSCALERIGHTCUBE1SWITCHCOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOROUTINERIGHTSTARTRIGHTSCALERIGHTCUBE1SWITCHCOMMANDGROUP_H_

#include "Commands/CommandGroup.h"

class AutoRoutineRightStartRightScaleRightCube1SwitchCommandGroup : public CommandGroup{
public:
	AutoRoutineRightStartRightScaleRightCube1SwitchCommandGroup() : CommandGroup("AutoRoutineRightStartRightScaleRightCube1SwitchCommandGroup"){
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(277.6, 19.5), Rotation2D::fromDegrees(0))));
		AddSequential(new AutoScaleCommandGroup("/home/lvuser/PathRightStartToRightScale.csv"));
		AddSequential(new AutoCubeCommandGroup("home/lvuser/PathRightScaleToRightCube1.csv", -1, -1));
		AddSequential(new AutoSwitchCommandGroup("/home/lvuser/PathRightCube1ToSwitch.csv", false));
	}
};

#endif /* SRC_COMMANDS_AUTOROUTINERIGHTSTARTRIGHTSCALERIGHTCUBE1SWITCHCOMMANDGROUP_H_ */
