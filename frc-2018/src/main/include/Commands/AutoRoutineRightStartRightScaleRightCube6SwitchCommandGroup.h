/*
 * AutoRoutineRightStartRightScaleLeftCube1LeftSwitchCommandGroup.h
 *
 *  Created on: Feb 20, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_AUTOROUTINERIGHTSTARTRIGHTSCALERIGHTCUBE6SWITCHCOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOROUTINERIGHTSTARTRIGHTSCALERIGHTCUBE6SWITCHCOMMANDGROUP_H_

#include "frc/Commands/CommandGroup.h"

class AutoRoutineRightStartRightScaleRightCube6SwitchCommandGroup : public CommandGroup{
public:
	AutoRoutineRightStartRightScaleRightCube6SwitchCommandGroup() : CommandGroup("AutoRoutineRightStartRightScaleRightCube6SwitchCommandGroup") {
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(277.6, 19.5), Rotation2D::fromDegrees(0))));
		AddSequential(new AutoScaleCommandGroup<ArmToMidScaleFront>("/home/lvuser/PathRightStartToRightScale.csv"));
		AddSequential(new AutoCubeCommandGroup("home/lvuser/PathRightScaleToRightCube6.csv", -1, -1));
		AddSequential(new AutoSwitchCommandGroup("/home/lvuser/PathRightCube6ToSwitch.csv", false, -1, 228));
	}
};

#endif /* SRC_COMMANDS_AUTOROUTINERIGHTSTARTRIGHTSCALERIGHTCUBE6SWITCHCOMMANDGROUP_H_ */
