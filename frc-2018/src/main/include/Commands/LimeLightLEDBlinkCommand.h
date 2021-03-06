#ifndef LimeLightLEDBlinkCommand_H
#define LimeLightLEDBlinkCommand_H

#include "CommandBase.h"

class LimeLightLEDBlinkCommand : public frc::InstantCommand {
public:
	LimeLightLEDBlinkCommand() : InstantCommand("LimeLightLEDBlinkCommand"){
	}

	void Initialize() {
		CommandBase::m_limeLight->BlinkLED();
	}
};

#endif  // LimeLightLEDBlinkCommand_H
