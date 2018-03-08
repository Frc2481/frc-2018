/*
 * AutoRLR.h
 *
 *  Created on: Mar 5, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOS_AUTORLR_H_
#define SRC_COMMANDS_AUTOS_AUTORLR_H_

#include "Commands/CommandGroup.h"

class AutoRLR : public CommandGroup{
public:
	AutoRLR() : CommandGroup("AutoRLR") {
		AddSequential(new ObserverResetPosCommand(RigidTransform2D(Translation2D(279.656, 16.5), Rotation2D::fromDegrees(0))));
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddSequential(new DriveTrainFollowPath("/home/lvuser/PathRightStartToRightScale.csv"));
		AddSequential(new PrintCommand("FollowPath"));

		AddParallel(new IntakeReleaseCubeCommandGroup(0.5), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		AddSequential(new WaitCommand(0.5));

		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new PrintCommand("ArmExtentionMotionScaling"));


		//cube
		AddParallel(new DriveTrainFollowPath("/home/lvuser/PathRightScaleToRightCube6.csv"));
		AddParallel(new ArmToSwitchBack(""), 1);

		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));

		AddSequential(new DriveTrainWaitForFieldXorYCommandGroup(155, -1));
		AddParallel(new ArmToIntakeBack(""), 1);

		AddSequential(new IntakeHasCubeCommand());
		AddSequential(new PrintCommand("Has Cube"));
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new PrintCommand("Clamp"));
		AddSequential(new DriveTrainStopCommand());
		AddSequential(new PrintCommand("StopDrive"));


		//switch
		AddSequential(new PrintCommand("StartSwitch"));
		AddSequential(new ArmToSwitchBack(""), 1.0);
		AddParallel(new DriveTrainFollowPath("/home/lvuser/PathRightCube6ToSwitch.csv"));
		AddSequential(new WaitCommand(0.5));
		AddSequential(new ArmWaitForPivotOnTargetCommand());
		AddParallel(new IntakeReleaseCubeCommandGroup(0.7));
		AddSequential(new WaitCommand(0.5));

		//cube 3
		AddParallel(new DriveTrainFollowPath("/home/lvuser/PathSwitchToRightCube5.csv"));
		AddSequential(new DriveTrainWaitForFieldYCommand(245)); // Wait to drive before lowering arm.
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

		AddParallel(new DriveTrainFollowPath("/home/lvuser/PathRightCube5ToRightScale.csv"));
		AddSequential(new ArmToStow(""));
	}
};

#endif /* SRC_COMMANDS_AUTOS_AUTORLR_H_ */
