#ifndef IntakeRollerLoadCommand_H
#define IntakeRollerLoadCommand_H

#include "../CommandBase.h"

class IntakeRollerLoadCommand : public InstantCommand {
public:
	IntakeRollerLoadCommand() : InstantCommand("IntakeRollerLoadCommand"){

	}

	void Initialize(){
		CommandBase::m_intake->RollerLoad();
	}

};

#endif  // IntakeRollerLoadCommand_H
