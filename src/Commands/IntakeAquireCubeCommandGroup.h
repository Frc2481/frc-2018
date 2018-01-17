#ifndef IntakeAquireCubeCommandGroup_H
#define IntakeAquireCubeCommandGroup_H

#include <Commands/CommandGroup.h>
#include "IntakeHasCubeCommand.h"
#include "IntakeRollerUnloadCommand.h"
#include "IntakeRollerLoadCommand.h"
#include "IntakeRollerOffCommand.h"
#include "IntakeClampOpenCommand.h"
#include "IntakeClampCloseCommand.h"

class IntakeAquireCubeCommandGroup : public CommandGroup {
public:
	IntakeAquireCubeCommandGroup() : CommandGroup("IntakeAquireCubeCommandGroup"){
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand());
		AddSequential(new IntakeHasCubeCommand());
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
	}
};
#endif  // IntakeAquireCubeCommandGroup_H
