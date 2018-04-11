/*
 * AutoLRR.h
 *
 *  Created on: Mar 3, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOS_AUTOLRR_H_
#define SRC_COMMANDS_AUTOS_AUTOLRR_H_

#include "Commands/CommandGroup.h"

class AutoLRR : public CommandGroup{
public:
	AutoLRR() : CommandGroup("AutoLRR") {
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.44, 19.5), Rotation2D::fromDegrees(0))));
//scale
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddSequential(new DriveTrainFollowPath("/home/lvuser/PathLeftStartToRightScale.csv"));
		AddSequential(new PrintCommand("FollwPath"));

		AddParallel(new IntakeReleaseCubeCommandGroup(0.75), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		AddSequential(new WaitCommand(0.5));

		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new PrintCommand("ArmExtentionMotionScaling"));

//cube 1
		AddParallel(new DriveTrainFollowPath("/home/lvuser/PathRightScaleToLeftCube6.csv"));
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

//switch 1
		AddSequential(new PrintCommand("StartSwitch"));
		AddSequential(new ArmToSwitchBack(""), 1.0);
		AddParallel(new DriveTrainFollowPath("/home/lvuser/PathLeftCube6ToSwitch2.csv"));
		AddSequential(new WaitCommand(0.5));
		AddSequential(new ArmWaitForPivotOnTargetCommand());
		AddParallel(new IntakeReleaseCubeCommandGroup(0.7));
		AddSequential(new WaitCommand(0.5));

//cube 2
		AddParallel(new DriveTrainFollowPath("/home/lvuser/PathSwitchToLeftCube52.csv"));
		AddSequential(new DriveTrainWaitForFieldYCommand(245)); // Wait to drive before lowing arm.
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
//		AddSequential(new ArmExtentionMotionScaling(0.7));
//
//		AddParallel(new ArmToMidScaleFront(""), 1.5);
//
		AddSequential(new DriveTrainFollowPath("/home/lvuser/PathLeftCube5ToRightScale.csv"));
		AddSequential(new PrintCommand("FollowPath"));
//
//		AddParallel(new IntakeReleaseCubeCommandGroup(1), 1.0);
//		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));
//
//		AddSequential(new WaitCommand(0.5));
//
		AddSequential(new ArmExtentionMotionScaling(1.0));
//		AddSequential(new PrintCommand("ArmExtentionMotionScaling"));
		AddSequential(new ArmToStow(""));
	}
};

#endif /* SRC_COMMANDS_AUTOS_AUTOLRR_H_ */
