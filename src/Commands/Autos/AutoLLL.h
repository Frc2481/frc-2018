/*
 * AutoLLL.h
 *
 *  Created on: Mar 3, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOS_AUTOLLL_H_
#define SRC_COMMANDS_AUTOS_AUTOLLL_H_

#include <Commands/CommandGroup.h>
#include "Commands/DriveTrainWaitForFinishedPathCommand.h"

class AutoLLL : public CommandGroup {
public:
	AutoLLL() {
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.44, 19.5), Rotation2D::fromDegrees(0))));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale1.csv"));
		AddSequential(new PrintCommand("FollowPath"));

		AddSequential(new DriveTrainWaitForFieldYCommand(226));
		AddParallel(new ArmToMidScaleFront(""), 1.5);

		AddSequential(new DriveTrainWaitForFieldYCommand(257));
		AddParallel(new IntakeReleaseCubeCommandGroup(1.0), 1.0);
		AddSequential(new WaitCommand(0.5));
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new PrintCommand("ArmExtentionMotionScaling"));

		// Cube 2
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale2.csv"));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new WaitCommand(1));
		AddSequential(new IntakeHasCubeCommand());
		AddSequential(new PrintCommand("Has Cube"));
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
//		AddSequential(new DriveTrainStopCommand());
//		AddSequential(new PrintCommand("StopDrive"));

		//Scale 2
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale3.csv"));
		AddSequential(new DriveTrainWaitForFieldYCommand(257));

		AddParallel(new IntakeReleaseCubeCommandGroup(1.0), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));
		AddSequential(new ArmExtentionMotionScaling(1.0));

		// Cube 3
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale4.csv"));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new WaitCommand(1));
		AddSequential(new IntakeHasCubeCommand());
		AddSequential(new PrintCommand("Has Cube"));
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
//		AddSequential(new DriveTrainStopCommand());
//		AddSequential(new PrintCommand("StopDrive"));

		//Scale 2
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale5.csv"));
		AddSequential(new DriveTrainWaitForFieldYCommand(257));

		AddParallel(new IntakeReleaseCubeCommandGroup(0.5), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));
		AddSequential(new ArmExtentionMotionScaling(1.0));

		// Cube 3
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale6.csv"));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new WaitCommand(1));
		AddSequential(new IntakeHasCubeCommand());
		AddSequential(new PrintCommand("Has Cube"));
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
//		AddSequential(new DriveTrainStopCommand());
//		AddSequential(new PrintCommand("StopDrive"));

		//Scale 3
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale7.csv"));
		AddSequential(new DriveTrainWaitForFieldYCommand(257));

		AddParallel(new IntakeReleaseCubeCommandGroup(0.75), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));
		AddSequential(new ArmExtentionMotionScaling(1.0));


		AddSequential(new ArmToStow(""));

//		AddSequential(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale1.csv"));
//		AddSequential(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale2.csv"));
//		AddSequential(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale3.csv"));
//		AddSequential(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale4.csv"));
//		AddSequential(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale5.csv"));
//		AddSequential(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale6.csv"));
//		AddSequential(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale7.csv"));



	}
};

#endif /* SRC_COMMANDS_AUTOS_AUTOLLL_H_ */
