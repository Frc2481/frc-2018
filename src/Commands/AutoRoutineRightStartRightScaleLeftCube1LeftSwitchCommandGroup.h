/*
 * AutoRoutineRightStartRightScaleLeftCube1LeftSwitchCommandGroup.h
 *
 *  Created on: Feb 20, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_AUTOROUTINERIGHTSTARTRIGHTSCALELEFTCUBE1LEFTSWITCHCOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOROUTINERIGHTSTARTRIGHTSCALELEFTCUBE1LEFTSWITCHCOMMANDGROUP_H_

#include "Commands/CommandGroup.h"

class AutoRoutineRightStartRightScaleLeftCube1LeftSwitchCommandGroup : public CommandGroup{
public:
	AutoRoutineRightStartRightScaleLeftCube1LeftSwitchCommandGroup() : CommandGroup("AutoRoutineRightStartRightScaleLeftCube1LeftSwitchCommandGroup") {
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(277.6, 19.5), Rotation2D::fromDegrees(0))));
		AddSequential(new AutoScaleCommandGroup("/home/lvuser/PathRightStartToRightScale.csv", "/home/lvuser/PathRightScaleBackUp.csv"));
		AddSequential(new AutoCubeCommandGroup("home/lvuser/PathRightScaleToLeftCube1.csv"));
		AddSequential(new AutoSwitchCommandGroup("/home/lvuser/PathLeftCube1ToLeftSwitch2.csv", false));
	}
};

#endif /* SRC_COMMANDS_AUTOROUTINERIGHTSTARTRIGHTSCALELEFTCUBE1LEFTSWITCHCOMMANDGROUP_H_ */
