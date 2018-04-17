/*
 * Auto3LLSwitch.h
 *
 *  Created on: Apr 13, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_AUTOS_AUTOL3LSWITCH_H_
#define SRC_COMMANDS_AUTOS_AUTOL3LSWITCH_H_

#include <Commands/CommandGroup.h>
#include "Commands/DriveTrainWaitForFinishedPathCommand.h"

class Auto3LLSwitch : public CommandGroup {
public:
	Auto3LLSwitch() : CommandGroup("Auto3LLSwitch") {
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.44, 19.5), Rotation2D::fromDegrees(0))));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_switch1.csv"));
		AddSequential(new PrintCommand("FollowPath"));
		AddParallel(new ArmToSwitchFront(""), 1.5);

		AddSequential(new DriveTrainWaitForFieldYCommand(135));

		AddParallel(new IntakeReleaseCubeCommandGroup(1.0), 1.0);
		AddSequential(new WaitCommand(0.5));
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		// Cube 2
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_switch2.csv"));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new WaitCommand(1));
		AddSequential(new IntakeHasCubeCommand());
		AddSequential(new PrintCommand("Has Cube"));
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));

		//Switch 2
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToSwitchBack(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_switch3.csv"));

		AddParallel(new IntakeReleaseCubeCommandGroup(1.0), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		// Cube 3
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_switch4.csv"));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new WaitCommand(1));
		AddSequential(new IntakeHasCubeCommand());
		AddSequential(new PrintCommand("Has Cube"));
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));

		//Switch 3
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToSwitchBack(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_switch5.csv"));

		AddParallel(new IntakeReleaseCubeCommandGroup(1.0), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		// Cube 3
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_switch6.csv"));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new WaitCommand(1));
		AddSequential(new IntakeHasCubeCommand());
		AddSequential(new PrintCommand("Has Cube"));
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));

		AddSequential(new ArmToStow(""));
	}
};

#endif /* SRC_COMMANDS_AUTOS_AUTOL3LSWITCH_H_ */
