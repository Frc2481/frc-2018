/*
 * AutoLL0.h
 *
 *  Created on: Mar 5, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOS_AUTOLL0_H_
#define SRC_COMMANDS_AUTOS_AUTOONLYLEFTSCALE_H_

#include "frc/Commands/CommandGroup.h"

class AutoLL0 : public CommandGroup{
public:
	AutoLL0() : CommandGroup("AutoLL0"){
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.4, 19.5), Rotation2D::fromDegrees(0))));
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddSequential(new DriveTrainFollowPath("/home/lvuser/PathLeftStartToLeftScale.csv"));
		AddSequential(new PrintCommand("FollowPath"));

		AddParallel(new IntakeReleaseCubeCommandGroup(0.5), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		AddSequential(new WaitCommand(0.5));

		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new PrintCommand("ArmExtentionMotionScaling"));

		AddSequential(new ArmToStow(""));
	}
};

#endif /* SRC_COMMANDS_AUTOS_AUTOLL0_H_ */
