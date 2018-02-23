/*
 * AutoRoutineRightStartRightScaleRightCube1RightSwitchCommandGroup.h
 *
 *  Created on: Feb 20, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_AUTOROUTINERIGHTSTARTRIGHTSCALERIGHTCUBE1RIGHTSWITCHCOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOROUTINERIGHTSTARTRIGHTSCALERIGHTCUBE1RIGHTSWITCHCOMMANDGROUP_H_

#include "Commands/CommandGroup.h"

class AutoRoutineRightStartRightScaleRightCube1RightSwitchCommandGroup : public CommandGroup{
public:
	AutoRoutineRightStartRightScaleRightCube1RightSwitchCommandGroup() : CommandGroup("AutoRoutineRightStartRightScaleRightCube1RightSwitchCommandGroup"){
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(277.6, 19.5), Rotation2D::fromDegrees(0))));
		AddSequential(new AutoScaleCommandGroup("/home/lvuser/PathRightStartToRightScale.csv", "/home/lvuser/PathRightScaleBackUp.csv"));
		AddSequential(new AutoCubeCommandGroup("home/lvuser/PathRightScaleToRightCube1.csv"));
		AddSequential(new AutoSwitchCommandGroup("/home/lvuser/PathRightCube1ToRightSwitch.csv", false));
	}
};

#endif /* SRC_COMMANDS_AUTOROUTINERIGHTSTARTRIGHTSCALERIGHTCUBE1RIGHTSWITCHCOMMANDGROUP_H_ */
