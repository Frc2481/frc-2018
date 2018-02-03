#ifndef LimeLightActivateRedScale_H
#define LimeLightActivateRedScale_H

#include "../CommandBase.h"

class LimeLightActivateRedScaleCommand : public InstantCommand {
public:
	LimeLightActivateRedScaleCommand() : InstantCommand("LimeLightActivateRedScaleCommand"){

	}

	void Initialize(){
	CommandBase :: m_limeLight->ActivateRedScale();
	}
};

#endif  // LimeLightActivateRedScale_H
