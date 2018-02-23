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
//#include "Robot.cpp"


class AutoRoutineCommandGroup : public CommandGroup{
public:
	AutoRoutineCommandGroup() : CommandGroup("AutoRoutineCommandGroup"){

//		AddSequential(new IntakeClampCloseCommand());
//		AddSequential(new ArmToStow(""));
//		AddSequential(task1); //fix
		AddSequential(new AutoScaleCommandGroup("/home/lvuser/PathLeftStartToLeftScale.csv", "/home/lvuser/PathLeftScaleBackUp.csv"));

		AddSequential(new DriveTrainFollowPath("home/lvuser/PathLeftScaleToLeftCube1.csv"));
//		AddSequential(new ArmToIntakeBack(""));
//		AddSequential(new IntakeAcquireCubeCommandGroup());
//		AddSequential(task2); //fix
//
		AddSequential(new DriveTrainFollowPath("home/lvuser/PathLeftCube1ToLeftScale.csv"));
//		AddSequential(new ArmToIntakeBack(""));
//		AddSequential(new IntakeAcquireCubeCommandGroup());
//		AddSequential(task3); //fix
	}
};

#endif /* SRC_COMMANDS_AUTOROUTINEGROUPCOMMAND_H_ */
