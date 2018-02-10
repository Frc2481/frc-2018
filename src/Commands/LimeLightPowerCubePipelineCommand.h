#ifndef LimeLightPowerCubePipelineCommand_H
#define LimeLightPowerCubePipelineCommand_H

#include "../CommandBase.h"

class LimeLightPowerCubePipelineCommand : public InstantCommand {
public:
	LimeLightPowerCubePipelineCommand() : InstantCommand("LimeLightPowerCubePipelineCommand"){
	}

	void Initialize(){
		CommandBase :: m_limeLight->PowerCubePipeline();
	}
};

#endif  // LimeLightPowerCubePipelineCommand_H
