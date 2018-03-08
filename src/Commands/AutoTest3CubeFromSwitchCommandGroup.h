/*
 * AutoTest3CubeFromSwitchCommandGroup.h
 *
 *  Created on: Mar 3, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOTEST3CUBEFROMSWITCHCOMMANDGROUP_H_
#define SRC_COMMANDS_AUTOTEST3CUBEFROMSWITCHCOMMANDGROUP_H_

#include "Commands/CommandGroup.h"
#include "Commands/ArmBaseCommand.h"

class AutoTest3CubeFromSwitchCommandGroup : public CommandGroup {
public:
	AutoTest3CubeFromSwitchCommandGroup() : CommandGroup("AutoTest3CubeFromSwitchCommandGroup") {
		AddSequential(new DriveTrainFollowPath("/home/lvuser/PathSwitchToLeftCube2.csv"));
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
		AddSequential(new ArmExtentionMotionScaling(0.3));
		AddSequential(new PrintCommand("ArmExtensionMotionScaling"));

		AddParallel(new ArmToMidScaleFront(""), 1.5);
		AddSequential(new PrintCommand("ARM"));

		AddSequential(new DriveTrainFollowPath("/home/lvuser/PathLeftCube2ToLeftScale.csv"));

		AddParallel(new IntakeReleaseCubeCommandGroup(0.5), 1.0);
		AddSequential(new PrintCommand("IntakeReleaseCubeCommandGroup"));

		AddSequential(new ArmExtentionMotionScaling(1.0));
		AddSequential(new PrintCommand("ArmExtentionMotionScaling"));
	}
};

#endif /* SRC_COMMANDS_AUTOTEST3CUBEFROMSWITCHCOMMANDGROUP_H_ */
