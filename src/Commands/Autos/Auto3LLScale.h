/*
 * Auto3LLScale.h
 *
 *  Created on: Mar 3, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOS_AUTOLLL_H_
#define SRC_COMMANDS_AUTOS_AUTOLLL_H_

#include <Commands/CommandGroup.h>
#include "Commands/DriveTrainWaitForFinishedPathCommand.h"
#include "Commands/DriveTrainWaitForHeadingCommand.h"
#include "Subsystems/Drivetrain.h"
#include "Commands/IntakeHasCubeSecurelyCommandGroup.h"
#include "Commands/DriveTrainSinusoidalJitter.h"

class Auto3LLScale : public CommandGroup {
public:
	Auto3LLScale() {
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.44, 19.5), Rotation2D::fromDegrees(0))));
		AddParallel(new ArmToStartPos(""));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale1.csv"));
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

		// Cube 2
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale2.csv"));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
		AddSequential(new WaitCommand(.25));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddSequential(new DriveTrainSinusoidalJitter(), 2.0);
		AddSequential(new PrintCommand("Has Cube"));
		AddParallel(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
		AddSequential(new IntakeRollerOffCommand());

		//Scale 2
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale3.csv"));
		AddSequential(new DriveTrainWaitForFieldYCommand(265));
		AddSequential(new WaitCommand(0.25));

		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerUnloadCommand(0.65));
		AddSequential(new WaitCommand(0.5));
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));
		AddSequential(new ArmExtentionMotionScaling(1.0));

		// Cube 3
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale4.csv"));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
		AddSequential(new WaitCommand(0.25));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddSequential(new DriveTrainSinusoidalJitter(), 2.0);
		AddSequential(new PrintCommand("Has Cube"));
		AddParallel(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
		AddSequential(new IntakeRollerOffCommand());

		//Scale 3
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale5.csv"));
		AddSequential(new DriveTrainWaitForFieldYCommand(265));
		AddSequential(new WaitCommand(0.25));

		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerUnloadCommand(0.9));
		AddSequential(new WaitCommand(0.5));
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));
		AddSequential(new ArmExtentionMotionScaling(1.0));

		// Cube 4
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale6.csv"));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
		AddSequential(new WaitCommand(0.25));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new WaitCommand(2));
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddSequential(new DriveTrainSinusoidalJitter(), 2.0);
		AddSequential(new PrintCommand("Has Cube"));
		AddParallel(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
		AddSequential(new IntakeRollerOffCommand());

		//Scale 4
		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_scale7.csv"));
		AddSequential(new ArmToStow(""));
	}
};

#endif /* SRC_COMMANDS_AUTOS_AUTOLLL_H_ */