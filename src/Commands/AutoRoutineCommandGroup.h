/*
 * AutoRoutineCommandGroup.h
 *
 *  Created on: Feb 8, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOROUTINEGROUPCOMMAND_H_
#define SRC_COMMANDS_AUTOROUTINEGROUPCOMMAND_H_

#include "Commands/CommandGroup.h"
#include "Commands/AutoRoutineCommandGroup.h"
#include "Commands/DriveTrainFollowPath.h"

class AutoRoutineCommandGroup : public CommandGroup{
public:
	AutoRoutineCommandGroup(Command* startPos, Command* task1, Command* task2, Command* task3) : CommandGroup("AutoRoutineCommandGroup"){
		AddSequential(startPos);
		AddSequential(new IntakeClampCloseCommand());
		if(task1 != nullptr) {
			AddSequential(task1);
			printf("task1");
		}
		if(task2 != nullptr) {
			AddSequential(task2);
			printf("task2");
		}
		if(task3 != nullptr) {
			AddSequential(task3);
			printf("task3");
		}
		AddSequential(new ArmToStow(""));
	}
};

#endif /* SRC_COMMANDS_AUTOROUTINEGROUPCOMMAND_H_ */
