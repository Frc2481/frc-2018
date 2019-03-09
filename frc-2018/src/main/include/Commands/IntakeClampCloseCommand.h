#ifndef IntakeClampCloseCommand_H
#define IntakeClampCloseCommand_H

#include "CommandBase.h"

class IntakeClampCloseCommand : public frc::InstantCommand {
public:
	IntakeClampCloseCommand() : InstantCommand("IntakeClampCloseCommand") {
	}

	void Initialize() {
		CommandBase::m_intake->CloseClamp();
	}
};

#endif  // IntakeClampCommand_H
