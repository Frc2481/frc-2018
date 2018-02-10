#ifndef IntakeAcquireCubeCommandGroup_H
#define IntakeAcquireCubeCommandGroup_H

#include <Commands/CommandGroup.h>
#include "IntakeHasCubeCommand.h"
#include "IntakeRollerUnloadCommand.h"
#include "IntakeRollerLoadCommand.h"
#include "IntakeRollerOffCommand.h"
#include "IntakeClampOpenCommand.h"
#include "IntakeClampCloseCommand.h"

class IntakeAcquireCubeCommandGroup : public CommandGroup {
public:
	IntakeAcquireCubeCommandGroup() : CommandGroup("IntakeAcquireCubeCommandGroup"){
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
		AddSequential(new IntakeHasCubeCommand());
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
	}
};
#endif  // IntakeAcquireCubeCommandGroup_H
