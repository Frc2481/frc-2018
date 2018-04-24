/*
 * AutoDriveToCenter.h
 *
 *  Created on: Apr 22, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_AUTOS_AUTODRIVETOCENTER_H_
#define SRC_COMMANDS_AUTOS_AUTODRIVETOCENTER_H_

#include "Commands/CommandGroup.h"

class AutoDriveToCenter : public CommandGroup{
public:
	AutoDriveToCenter() : CommandGroup("AutoDriveToCenter") {
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.44, 19.5), Rotation2D::fromDegrees(0))));
		AddParallel(new ArmToStow(""));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_platform_center1.csv"));
	}
};

#endif /* SRC_COMMANDS_AUTOS_AUTODRIVETOCENTER_H_ */
