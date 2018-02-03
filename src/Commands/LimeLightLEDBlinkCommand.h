#ifndef LimeLightLEDBlinkCommand_H
#define LimeLightLEDBlinkCommand_H

#include "../CommandBase.h"

class LimeLightLEDBlinkCommand : public InstantCommand {
public:
	LimeLightLEDBlinkCommand() : InstantCommand("LimeLightLEDBlinkCommand"){

	}
	void Initialize(){
		CommandBase :: m_limeLight->BlinkLight();
	}

};

#endif  // LimeLightLEDBlinkCommand_H
