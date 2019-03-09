#ifndef IntakeAcquireCubeCommandGroup_H
#define IntakeAcquireCubeCommandGroup_H

#include <frc/Commands/CommandGroup.h>
#include "IntakeHasCubeCommand.h"
#include "IntakeRollerUnloadCommand.h"
#include "IntakeRollerLoadCommand.h"
#include "IntakeRollerOffCommand.h"
#include "IntakeClampOpenCommand.h"
#include "IntakeClampCloseCommand.h"
#include "Commands/ArmBaseCommand.h"
#include "Subsystems/Intake.h"

class IntakeAcquireCubeCommandGroup : public frc::CommandGroup {
public:
	IntakeAcquireCubeCommandGroup() : CommandGroup("IntakeAcquireCubeCommandGroup"){
		Requires(CommandBase::m_intake.get());
		AddSequential(new IntakeClampOpenCommand());
		AddSequential(new IntakeRollerLoadCommand(1));
		AddSequential(new IntakeHasCubeCommand());
		AddSequential(new IntakeRollerOffCommand());
		AddSequential(new IntakeClampCloseCommand());
//		AddSequential(new ArmToStow(""));
	}
};
#endif  // IntakeAcquireCubeCommandGroup_H
