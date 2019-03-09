/*
 * AutoRRR.h
 *
 *  Created on: Mar 5, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOS_AUTORRR_H_
#define SRC_COMMANDS_AUTOS_AUTORRR_H_

#include "frc/Commands/CommandGroup.h"

class AutoRRR : public CommandGroup{
public:
	AutoRRR() : CommandGroup("AutoRRR") {
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(279.656, 16.5), Rotation2D::fromDegrees(0))));
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront(""), 1.5);

		AddSequential(new DriveTrainFollowPath("/home/lvuser/PathRightStartToRightScale.csv"));
		AddSequential(new PrintCommand("FollowPath"));

		AddParallel(new IntakeReleaseCubeCommandGroup(0.5), 1.0);
		AddSequential(new WaitCommand(0.5));
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new PrintCommand("ArmExtentionMotionScaling"));

		// Cube 2
		AddParallel(new DriveTrainFollowPath("/home/lvuser/PathRightScaleToRightCube1.csv"));
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
		AddParallel(new ArmToIntakeBack(""), 1);
		AddSequential(new IntakeHasCubeCommand());
		AddSequential(new PrintCommand("Has Cube"));
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
		AddSequential(new DriveTrainStopCommand());
		AddSequential(new PrintCommand("StopDrive"));

		// Switch 2
		AddSequential(new PrintCommand("StartSwitch"));
		AddSequential(new ArmToSwitchBack(""), 1.0);
		AddParallel(new DriveTrainFollowPath("/home/lvuser/PathRightCube1ToSwitch.csv"));
		AddSequential(new WaitCommand(0.5));
		AddSequential(new ArmWaitForPivotOnTargetCommand());
		AddParallel(new IntakeReleaseCubeCommandGroup(0.7));

		//cube 3
		AddParallel(new DriveTrainFollowPath("/home/lvuser/PathSwitchToRightCube2.csv"));
		AddSequential(new DriveTrainWaitForFieldYCommand(245)); // Wait to drive before lowing arm.
		AddParallel(new ArmToIntakeBack(""), 1);
		AddParallel(new IntakeClampOpenCommand());
		AddParallel(new IntakeRollerLoadCommand(1));
		AddSequential(new IntakeHasCubeCommand());
		AddSequential(new PrintCommand("Has Cube"));
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
		AddSequential(new DriveTrainStopCommand());
		AddSequential(new PrintCommand("StopDrive"));

		// Scale 2
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddSequential(new DriveTrainFollowPath("/home/lvuser/PathRightCube2ToRightScale.csv"));

		AddParallel(new IntakeReleaseCubeCommandGroup(0.5), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		AddSequential(new WaitCommand(0.5));

		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new PrintCommand("ArmExtentionMotionScaling"));

		AddSequential(new ArmToStow(""));
	}
};

#endif /* SRC_COMMANDS_AUTOS_AUTORRR_H_ */
