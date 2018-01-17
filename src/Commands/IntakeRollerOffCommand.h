#ifndef IntakeRollerOffCommand_H
#define IntakeRollerOffCommand_H

#include "../CommandBase.h"

class IntakeRollerOffCommand : public InstantCommand {
public:
	IntakeRollerOffCommand() : InstantCommand ("IntakeRollerOffCommand"){

	}

	void Initialize(){
		CommandBase::m_intake->RollerOff();
	}

};

#endif  // RollerOffCommand_H
