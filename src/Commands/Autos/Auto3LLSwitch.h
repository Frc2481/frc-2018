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
#include "Commands/DriveTrainSinusoidalJitter.h"
#include "Commands/DriveTrainSetBrakeCommand.h"

class Auto3LLSwitch : public CommandGroup {
public:
	Auto3LLSwitch() : CommandGroup("Auto3LLSwitch") {
		//Switch 1
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.44, 19.5), Rotation2D::fromDegrees(0))));
		AddSequential(new DriveTrainSetBrakeCommand(true));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_switch1.csv"));
		AddSequential(new PrintCommand("FollowPath"));
		AddSequential(new DriveTrainWaitForFieldYCommand(30));
		AddParallel(new ArmToSwitchFront(""), 1.5);

		AddSequential(new DriveTrainWaitForFinishedPathCommand());

		AddParallel(new IntakeReleaseCubeCommandGroup(1.0), 1.0);
		AddSequential(new WaitCommand(0.5));
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		// Cube 2
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_switch2.csv"));
		AddParallel(new IntakeClampOpenCommand());
		AddParallel(new IntakeRollerLoadCommand(1));
		AddSequential(new WaitCommand(0.25));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddSequential(new DriveTrainSinusoidalJitter(false), 2.0);
		AddSequential(new PrintCommand("Has Cube"));
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
		AddSequential(new IntakeRollerOffCommand());

		//Switch 2
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToSwitchBack(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_switch3.csv"));
		AddSequential(new WaitCommand(1.0));

		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddSequential(new IntakeReleaseCubeCommandGroup(1.0), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		// Cube 3
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_switch4.csv"));
		AddParallel(new IntakeClampOpenCommand());
		AddParallel(new IntakeRollerLoadCommand(1));
		AddSequential(new WaitCommand(0.25));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddSequential(new DriveTrainSinusoidalJitter(false), 2.0);
		AddSequential(new PrintCommand("Has Cube"));
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
		AddSequential(new IntakeRollerOffCommand());

		//Switch 3
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToSwitchBack(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_switch5.csv"));
		AddSequential(new WaitCommand(1.0));

		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddSequential(new IntakeReleaseCubeCommandGroup(1.0), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		// Cube 3
		AddParallel(new DriveTrainFollowPath("/home/lvuser/Path_LL_switch6.csv"));
		AddParallel(new IntakeClampOpenCommand());
		AddParallel(new IntakeRollerLoadCommand(1));
		AddSequential(new WaitCommand(0.25));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new DriveTrainWaitForFinishedPathCommand());
		AddSequential(new DriveTrainSinusoidalJitter(false), 2.0);
		AddSequential(new PrintCommand("Has Cube"));
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
		AddSequential(new IntakeRollerOffCommand());

		//Switch 4
		AddSequential(new ArmToStow(""));
	}
};

#endif /* SRC_COMMANDS_AUTOS_AUTOL3LSWITCH_H_ */
