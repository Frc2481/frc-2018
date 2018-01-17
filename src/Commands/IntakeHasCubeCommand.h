#ifndef IntakeCubeHasCommand_H
#define IntakeCubeHasCommand_H

#include "../CommandBase.h"

class IntakeHasCubeCommand : public InstantCommand {
public:
	IntakeHasCubeCommand() : InstantCommand("IntakeHasCubeCommand"){

	}

	bool IsFinished(){
		return CommandBase::m_intake->HasCube();
	}
};

#endif  // IntakeCubeHasCommand_H
