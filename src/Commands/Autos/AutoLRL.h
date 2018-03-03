/*
 * AutoLRL.h
 *
 *  Created on: Mar 3, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOS_AUTOLRL_H_
#define SRC_COMMANDS_AUTOS_AUTOLRL_H_

#include "Commands/CommandGroup.h"

class AutoLRL : public CommandGroup{
public:
	AutoLRL() : CommandGroup("AutoLRL") {
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront, 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddSequential(new DriveTrainFollowPath("/home/lvuser/PathLeftStartToLeftScale.csv"));
		AddSequential(new PrintCommand("FollowPath"));

		AddParallel(new IntakeReleaseCubeCommandGroup(0.5), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new PrintCommand("ArmExtentionMotionScaling"));


		//cube
		AddParallel(new DriveTrainFollowPath("/home/lvuser/PathLeftScaleToLeftCube6.csv"));
		AddParallel(new ArmToSwitchBack(""), 1);

//		AddParallel(new IntakeAcquireCubeCommandGroup());

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


	}
};

#endif /* SRC_COMMANDS_AUTOS_AUTOLRL_H_ */
