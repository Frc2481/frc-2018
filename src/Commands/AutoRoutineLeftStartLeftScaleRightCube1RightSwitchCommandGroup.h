/*
 * AutoRoutineLeftStartLeftScaleRightCube1RightSwitchCommandGroup.h
 *
 *  Created on: Feb 19, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_AUTOROUTINELEFTSTARTLEFTSCALERIGHTCUBE1RIGHTSWITCHCOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOROUTINELEFTSTARTLEFTSCALERIGHTCUBE1RIGHTSWITCHCOMMANDGROUP_H_

#include "Commands/CommandGroup.h"

class AutoRoutineLeftStartLeftScaleRightCube1RightSwitchCommandGroup : public CommandGroup{
public:
	AutoRoutineLeftStartLeftScaleRightCube1RightSwitchCommandGroup() : CommandGroup("AutoRoutineLeftStartLeftScaleRightCube1RightSwitchCommandGroup") {
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.4, 19.5), Rotation2D::fromDegrees(0))));
		AddSequential(new AutoScaleCommandGroup("/home/lvuser/PathLeftStartToLeftScale.csv", "/home/lvuser/PathLeftScaleBackUp.csv"));
		AddSequential(new AutoCubeCommandGroup("home/lvuser/PathLeftScaleToRightCube1.csv"));
		AddSequential(new AutoSwitchCommandGroup("/home/lvuser/PathRightCube1ToRightSwitch2.csv", false));
	}
};

#endif /* SRC_COMMANDS_AUTOROUTINELEFTSTARTLEFTSCALERIGHTCUBE1RIGHTSWITCHCOMMANDGROUP_H_ */
