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
			AddSequential(new PrintCommand("Task 1 Start"));
			AddSequential(task1);
			AddSequential(new PrintCommand("Task 1 Finished"));
		}
		if(task2 != nullptr) {
			AddSequential(new PrintCommand("Task 2 Start"));
			AddSequential(task2);
			AddSequential(new PrintCommand("Task 2 Finished"));
		}
		if(task3 != nullptr) {
			AddSequential(new PrintCommand("Task 3 Start"));
			AddSequential(task3);
			AddSequential(new PrintCommand("Task 3 Finished"));
		}
		AddSequential(new ArmToStow(""));
	}
};

#endif /* SRC_COMMANDS_AUTOROUTINEGROUPCOMMAND_H_ */
