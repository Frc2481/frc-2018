/*
 * AutoLLR.h
 *
 *  Created on: Mar 3, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOS_AUTOLLR_H_
#define SRC_COMMANDS_AUTOS_AUTOLLR_H_

#include "frc/Commands/CommandGroup.h"

class AutoLLR : public CommandGroup{
public:
	AutoLLR() : CommandGroup("AutoLLR") {
		//cube 1 to switch
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.4, 19.5), Rotation2D::fromDegrees(0))));
		AddParallel(new DriveTrainFollowPath("/home/lvuser/PathLeftStartToLeftCube4.csv"));
		AddSequential(new DriveTrainWaitForFieldYCommand(35));
		AddParallel(new ArmToSwitchFront(""), 1.0);
		AddSequential(new DriveTrainWaitForFieldXorYCommandGroup(-1, 135));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerUnloadCommand(0.7));
		AddSequential(new WaitCommand(0.5));

		//cube 4
//		AddParallel(new DriveTrainFollowPath("/home/lvuser/PathLeftSwitchToLeftCube4.csv"));
		AddParallel(new ArmToSwitchBack(""), 1);

//		AddParallel(new IntakeAcquireCubeCommandGroup());

		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));

		AddSequential(new DriveTrainWaitForFieldXorYCommandGroup(105, -1));
		AddParallel(new ArmToIntakeBack(""), 1);

		AddSequential(new IntakeHasCubeCommand());
		AddSequential(new PrintCommand("Has Cube"));
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
		AddSequential(new DriveTrainStopCommand());
		AddSequential(new PrintCommand("StopDrive"));
		AddSequential(new WaitCommand(0.25));

//scale 1
		AddSequential(new ArmExtentionMotionScaling(0.4));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddSequential(new DriveTrainFollowPath("/home/lvuser/PathLeftCube4ToRightScale.csv"));
		AddSequential(new PrintCommand("FollowPath"));

		AddParallel(new IntakeReleaseCubeCommandGroup(1.0), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new PrintCommand("ArmExtentionMotionScaling"));

//Cube 6
		AddParallel(new DriveTrainFollowPath("/home/lvuser/PathRightScaleToLeftCube62.csv"));
		AddSequential(new WaitCommand(0.5)); // Don't clip the scale with the intake.
		AddParallel(new IntakeClampOpenCommand());
		AddParallel(new IntakeRollerLoadCommand(1));

		AddParallel(new ArmToIntakeBack(""), 1);

		AddSequential(new IntakeHasCubeCommand());
		AddSequential(new PrintCommand("Has Cube"));
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
		AddSequential(new DriveTrainStopCommand());
		AddSequential(new PrintCommand("StopDrive"));
		AddSequential(new WaitCommand(0.25));

//scale 2
		AddSequential(new ArmExtentionMotionScaling(0.7));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddSequential(new DriveTrainFollowPath("/home/lvuser/PathLeftCube6ToRightScale.csv"));
		AddSequential(new PrintCommand("FollowPath"));

		AddParallel(new IntakeReleaseCubeCommandGroup(0.4), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		AddSequential(new WaitCommand(1));

		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new PrintCommand("ArmExtentionMotionScaling"));

		AddSequential(new ArmToStow(""));

	}
};

#endif /* SRC_COMMANDS_AUTOS_AUTOLLR_H_ */
