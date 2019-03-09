/*
 * TestArmDuringPathCommandGroup.h
 *
 *  Created on: Feb 23, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_TESTARMDURINGPATHCOMMANDGROUP_H_
#define SRC_COMMANDS_TESTARMDURINGPATHCOMMANDGROUP_H_

#include "frc/Commands/CommandGroup.h"
#include "DriveTrainWaitForFieldYCommand.h"
#include "DriveTrainFollowPath.h"

class TestArmDuringPathCommandGroup : public frc::CommandGroup{
public:
	TestArmDuringPathCommandGroup() : CommandGroup("TestArmDuringPathCommandGroup") {
		AddParallel(new DriveTrainFollowPath("/home/lvuser/robotPath.csv"));
		AddSequential(new DriveTrainWaitForFieldYCommand(50));
		AddSequential(new ArmToLowScaleFront(""));

	}
};

#endif /* SRC_COMMANDS_TESTARMDURINGPATHCOMMANDGROUP_H_ */
