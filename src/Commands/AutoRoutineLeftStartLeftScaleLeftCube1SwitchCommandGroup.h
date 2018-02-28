/*
 * AutoRoutineLeftStartLeftScaleLeftCube1SwitchCommandGroup.h
 *
 *  Created on: Feb 19, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_AUTOROUTINELEFTSTARTLEFTSCALELEFTCUBE1SWITCHCOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOROUTINELEFTSTARTLEFTSCALELEFTCUBE1SWITCHCOMMANDGROUP_H_

#include "Commands/CommandGroup.h"
#include "ObserverResetPosCommand.h"
#include "AutoCubeCommandGroup.h"
#include "AutoSwitchCommandGroup.h"
#include "AutoScaleCommandGroup.h"

class AutoRoutineLeftStartLeftScaleLeftCube1SwitchCommandGroup : public CommandGroup{
public:
	AutoRoutineLeftStartLeftScaleLeftCube1SwitchCommandGroup() : CommandGroup("AutoRoutineLeftStartLeftScaleLeftCube1SwitchCommandGroup"){
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.4, 19.5), Rotation2D::fromDegrees(0))));
		AddSequential(new AutoScaleCommandGroup("/home/lvuser/PathLeftStartToLeftScale.csv"));
		AddSequential(new AutoCubeCommandGroup("home/lvuser/PathLeftScaleToLeftCube1.csv", -1, -1));
		AddSequential(new AutoSwitchCommandGroup("/home/lvuser/PathLeftCube1ToSwitch.csv", false, -1, 228));
	}
};

#endif /* SRC_COMMANDS_AUTOROUTINELEFTSTARTLEFTSCALELEFTCUBE1SWITCHCOMMANDGROUP_H_ */
