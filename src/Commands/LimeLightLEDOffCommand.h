#ifndef TurnLEDOffCommand_H
#define TurnLEDOffCommand_H

#include "../CommandBase.h"

class TurnLEDOffCommand : public InstantCommand {
public:
	TurnLEDOffCommand() : InstantCommand ("TurnLEDOffCommand") {

	}
	void Initialize(){
		CommandBase::m_limeLight->TurnOffLED();
	}

};

#endif  // TurnLEDOffCommand_H
