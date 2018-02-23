/*
 * AutoRoutineLeftStartLeftScaleLeftCube1LeftSwitchCommandGroup.h
 *
 *  Created on: Feb 19, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_AUTOROUTINELEFTSTARTLEFTSCALELEFTCUBE1LEFTSWITCHCOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOROUTINELEFTSTARTLEFTSCALELEFTCUBE1LEFTSWITCHCOMMANDGROUP_H_

#include "Commands/CommandGroup.h"

class AutoRoutineLeftStartLeftScaleLeftCube1LeftSwitchCommandGroup : public CommandGroup{
public:
	AutoRoutineLeftStartLeftScaleLeftCube1LeftSwitchCommandGroup() : CommandGroup("AutoRoutineLeftStartLeftScaleLeftCube1LeftSwitchCommandGroup"){
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.4, 19.5), Rotation2D::fromDegrees(0))));
		AddSequential(new AutoScaleCommandGroup("/home/lvuser/PathLeftStartToLeftScale.csv", "/home/lvuser/PathLeftScaleBackUp.csv"));
		AddSequential(new AutoCubeCommandGroup("home/lvuser/PathLeftScaleToLeftCube1.csv"));
		AddSequential(new AutoSwitchCommandGroup("/home/lvuser/PathLeftCube1ToLeftSwitch.csv", false));
	}
};

#endif /* SRC_COMMANDS_AUTOROUTINELEFTSTARTLEFTSCALELEFTCUBE1LEFTSWITCHCOMMANDGROUP_H_ */
