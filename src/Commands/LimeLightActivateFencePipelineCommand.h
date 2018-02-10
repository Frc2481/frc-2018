#ifndef LimeLightActivateFencePipelineCommand_H
#define LimeLightActivateFencePipelineCommand_H

#include "../CommandBase.h"

class LimeLightActivateFencePipelineCommand : public InstantCommand {
public:
	LimeLightActivateFencePipelineCommand() : InstantCommand("LimeLightActivateFencePipeLineCommand"){
	}

	void Initialize(){
		CommandBase :: m_limeLight->ActivateFencePipeline();
	}
};

#endif  // LimeLightActivateFencePipelineCommand_H
