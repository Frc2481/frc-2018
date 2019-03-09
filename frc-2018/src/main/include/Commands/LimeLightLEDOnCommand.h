#ifndef TurnLEDOnCommand_H
#define TurnLEDOnCommand_H

#include "CommandBase.h"

class TurnLEDOnCommand : public frc::InstantCommand  {
public:
	TurnLEDOnCommand() : InstantCommand ("TurnLEDOnCommand"){
	}

	void Initialize() {
		CommandBase::m_limeLight->TurnOnLED();
	}
};

#endif  // TurnLEDOnCommand_H
