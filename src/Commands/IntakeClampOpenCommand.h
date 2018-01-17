#ifndef IntakeClampOpenCommand_H
#define IntakeClampOpenCommand_H

#include "../CommandBase.h"

class IntakeClampOpenCommand : public InstantCommand {
public:
	IntakeClampOpenCommand() : InstantCommand("IntakeClampOpenCommand"){

	}

	void Initialize(){
		CommandBase::m_intake->OpenClamp();
	}

};

#endif  // IntakeClampOpenCommand_H
