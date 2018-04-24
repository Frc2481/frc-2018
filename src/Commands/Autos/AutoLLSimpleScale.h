/*
 * AutoLLSimpleScale.h
 *
 *  Created on: Apr 22, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_AUTOS_AUTOLLSIMPLESCALE_H_
#define SRC_COMMANDS_AUTOS_AUTOLLSIMPLESCALE_H_

#include "Commands/CommandGroup.h"

class AutoLLSimpleScale : public CommandGroup {
public:
	AutoLLSimpleScale() : CommandGroup("AutoLLSimpleScale") {
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.44, 19.5), Rotation2D::fromDegrees(0))));
		AddParallel(new ArmToStartPos(""));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_L_scale_simple1.csv"));
		AddSequential(new PrintCommand("FollowPath"));

		AddSequential(new DriveTrainWaitForFieldYCommand(190));
		AddParallel(new ArmToMidScaleFront(""), 1.5);

		AddSequential(new DriveTrainWaitForFieldYCommand(265));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerUnloadCommand(0.77));
		AddSequential(new WaitCommand(0.5));
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new PrintCommand("ArmExtentionMotionScaling"));

		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_L_scale_simple2.csv"));
	}
};

#endif /* SRC_COMMANDS_AUTOS_AUTOLLSIMPLESCALE_H_ */
