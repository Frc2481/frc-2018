/*
 * AutoRoutineCommandGroup.h
 *
 *  Created on: Feb 8, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_AUTOROUTINEGROUPCOMMAND_H_
#define SRC_COMMANDS_AUTOROUTINEGROUPCOMMAND_H_

#include "CommandBase.h"
#include "Robot.cpp"


class AutoRoutineCommandGroup : public CommandGroup{
public:
	AutoRoutineCommandGroup(Autos task1, Autos task2, Autos task3) : CommandGroup("AutoRoutineCommandGroup"){
		AddSequential(new IntakeClampCloseCommand());
		AddSequential(new ArmToStow(""));
		AddSequential(task1); //fix

		AddSequential(new DriveTrainFollowPath("path to cube"));
		AddSequential(new ArmToIntakeBack(""));
		AddSequential(new IntakeAcquireCubeCommandGroup());
		AddSequential(task2); //fix

		AddSequential(new DriveTrainFollowPath("path to cube"));
		AddSequential(new ArmToIntakeBack(""));
		AddSequential(new IntakeAcquireCubeCommandGroup());
		AddSequential(task3); //fix
	}
};

#endif /* SRC_COMMANDS_AUTOROUTINEGROUPCOMMAND_H_ */
