/*
 * AutoRoutineLeftStartLeftScaleLeftCube6SwitchCommandGroup.h
 *
 *  Created on: Feb 19, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_AUTOROUTINELEFTSTARTLEFTSCALELEFTCUBE6SWITCHCOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOROUTINELEFTSTARTLEFTSCALELEFTCUBE6SWITCHCOMMANDGROUP_H_

#include "Commands/CommandGroup.h"
#include "Commands/AutoScaleCommandGroup.h"
#include "Commands/AutoCubeCommandGroup.h"
#include "Commands/AutoSwitchCommandGroup.h"
#include "ObserverResetPosCommand.h"

class AutoRoutineLeftStartLeftScaleLeftCube6SwitchCommandGroup : public CommandGroup{
public:
	AutoRoutineLeftStartLeftScaleLeftCube6SwitchCommandGroup() : CommandGroup("AutoRoutineLeftStartLeftScaleLeftCube6SwitchCommandGroup") {
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.4, 19.5), Rotation2D::fromDegrees(0))));
		AddSequential(new AutoScaleCommandGroup<ArmToMidScaleFront>("/home/lvuser/PathLeftStartToLeftScale.csv"));
		AddSequential(new AutoCubeCommandGroup("/home/lvuser/PathLeftScaleToLeftCube6.csv", 155, -1));
//		AddSequential(new AutoCubeCommandGroup("/home/lvuser/PathLeftScaleToLeftCube6.csv", -1, -1));
		AddSequential(new AutoSwitchCommandGroup("/home/lvuser/PathLeftCube6ToSwitch.csv", false, -1, 228));
	}
};

#endif /* SRC_COMMANDS_AUTOROUTINELEFTSTARTLEFTSCALELEFTCUBE6RIGHTSWITCHCOMMANDGROUP_H_ */
