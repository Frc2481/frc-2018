/*
 * Auto3LRScale.h
 *
 *  Created on: Apr 15, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_AUTOS_AUTO3RSCALE_H_
#define SRC_COMMANDS_AUTOS_AUTO3RSCALE_H_

#include <Commands/CommandGroup.h>
#include "Commands/DriveTrainWaitForFinishedPathCommand.h"
#include "Commands/IntakeHasCubeSecurelyCommandGroup.h"
#include "Commands/DriveTrainWaitForHeadingCommand.h"

class Auto3LRScale : public CommandGroup {
public:
	Auto3LRScale() : CommandGroup("Auto3LRScale") {
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.44, 19.5), Rotation2D::fromDegrees(0))));
		AddParallel(new ArmToStartPos(""));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LR_scale1.csv"));
		AddSequential(new PrintCommand("FollowPath"));

		AddSequential(new DriveTrainWaitForFieldXCommand(245));
		AddParallel(new ArmToMidScaleFront(""), 1.5);

		AddSequential(new DriveTrainWaitForFieldYCommand(265));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerUnloadCommand(0.7));
		AddSequential(new WaitCommand(0.75));
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new PrintCommand("ArmExtentionMotionScaling"));

		// Cube 2
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LR_scale2.csv"));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new WaitCommand(1));
		AddSequential(new IntakeHasCubeCommand(), 3.0);
		AddSequential(new PrintCommand("Has Cube"));
		AddParallel(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
		AddSequential(new IntakeRollerOffCommand());

		//Scale 2
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

//		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LR_scale3.csv"));
		AddSequential(new DriveTrainWaitForFieldYCommand(265));

		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerUnloadCommand(0.5));
		AddSequential(new WaitCommand(0.75));
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));
		AddSequential(new ArmExtentionMotionScaling(1.0));

		// Cube 3
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LR_scale4.csv"));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new WaitCommand(2));
		AddSequential(new IntakeHasCubeCommand(), 3.0);
		AddSequential(new PrintCommand("Has Cube"));
		AddParallel(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
		AddSequential(new IntakeRollerOffCommand());

		//Scale 3
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

//		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LR_scale5.csv"));
		AddSequential(new DriveTrainWaitForFieldYCommand(265));

		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerUnloadCommand(0.5));
		AddSequential(new WaitCommand(0.75));
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));
		AddSequential(new ArmExtentionMotionScaling(1.0));

		// Cube 4
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LR_scale6.csv"));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new WaitCommand(2));
		AddSequential(new IntakeHasCubeCommand(), 3.0);
		AddSequential(new PrintCommand("Has Cube"));
		AddParallel(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
		AddSequential(new IntakeRollerOffCommand());

		//Scale 4
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToLowScale2Front(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

//		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LR_scale7.csv"));
		AddSequential(new DriveTrainWaitForFieldYCommand(265));
		AddSequential(new PrintCommand("past y point scale path 7\n"));
//		AddSequential(new DriveTrainWaitForHeadingCommand(Rotation2D::fromDegrees(-15)));
//		AddSequential(new PrintCommand("at heading scale path 7\n"));

		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerUnloadCommand(0.7));
		AddSequential(new WaitCommand(1.5));
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));
		AddSequential(new ArmExtentionMotionScaling(1.0));

		AddSequential(new ArmToStow(""));
	}
};

#endif /* SRC_COMMANDS_AUTOS_AUTO3RSCALE_H_ */
