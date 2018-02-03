#ifndef ActivateFencePipelineCommand_H
#define ActivateFencePipelineCommand_H

#include "../CommandBase.h"

class ActivateFencePipelineCommand : public InstantCommand {
public:
	ActivateFencePipelineCommand() : InstantCommand("ActivateFencePipelineCommand"){

	}
	void Initialize(){
	CommandBase :: m_limeLight->ActivateFencePipeline();
	}

};

#endif  // ActivateFencePipelineCommand_H
