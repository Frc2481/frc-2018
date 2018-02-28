/*
 * DriveTrainWaitForFieldXorYCommandGroup.h
 *
 *  Created on: Feb 23, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_DRIVETRAINWAITFORFIELDXORYCOMMANDGROUP_H_
#define SRC_COMMANDS_DRIVETRAINWAITFORFIELDXORYCOMMANDGROUP_H_

#include <Commands/CommandGroup.h>
#include "DriveTrainWaitForFieldXCommand.h"
#include "DriveTrainWaitForFieldYCommand.h"

class DriveTrainWaitForFieldXorYCommandGroup : public CommandGroup{
public:
	DriveTrainWaitForFieldXorYCommandGroup(int x, int y) : CommandGroup("DriveTrainWaitForFieldXorYCommandGroup") {
		if(x != -1) {
			AddSequential(new DriveTrainWaitForFieldXCommand(x));
		} else if (y != -1) {
			AddSequential(new DriveTrainWaitForFieldYCommand(y));
		}
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINWAITFORFIELDXORYCOMMANDGROUP_H_ */
