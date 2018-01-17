#ifndef IntakeRollerUnloadCommand_H
#define IntakeRollerUnloadCommand_H

#include "../CommandBase.h"

class IntakeRollerUnloadCommand : public InstantCommand {
public:
	IntakeRollerUnloadCommand() : InstantCommand("IntakeRollerUnloadCommand"){

	}
	void Initialize(){
		CommandBase::m_intake->RollerUnload();
	}

};

#endif  // IntakeRollerUnloadCommand_H

