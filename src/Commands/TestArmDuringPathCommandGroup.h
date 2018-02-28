/*
 * TestArmDuringPathCommandGroup.h
 *
 *  Created on: Feb 23, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_TESTARMDURINGPATHCOMMANDGROUP_H_
#define SRC_COMMANDS_TESTARMDURINGPATHCOMMANDGROUP_H_

#include <Commands/CommandGroup.h>
#include "DriveTrainWaitForFieldYCommand.h"

class TestArmDuringPathCommandGroup : public CommandGroup{
public:
	TestArmDuringPathCommandGroup() : CommandGroup("TestArmDuringPathCommandGroup") {
		AddParallel(new DriveTrainFollowPath("/home/lvuser/robotPath.csv"));
		AddSequential(new DriveTrainWaitForFieldYCommand(50));
		AddSequential(new ArmToLowScaleFront(""));

	}
};

#endif /* SRC_COMMANDS_TESTARMDURINGPATHCOMMANDGROUP_H_ */
