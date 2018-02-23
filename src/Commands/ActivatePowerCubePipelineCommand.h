#ifndef ActivatePowerCubePipelineCommand_H
#define ActivatePowerCubePipelineCommand_H

#include "../CommandBase.h"

class ActivatePowerCubePipelineCommand : public InstantCommand {
public:
	ActivatePowerCubePipelineCommand() : InstantCommand("ActivatePowerCubePipelineCommand"){
	}

	void Initialize() {
		CommandBase::m_limeLight->ActivatePowerCubePipeline();
	}
};

#endif  // ActivatePowerCubePipelineCommand_H
