#ifndef LimeLightActivateBlueScale_H
#define LimeLightActivateBlueScale_H

#include "../CommandBase.h"

class LimeLightActivateBlueScaleCommand : public InstantCommand {
public:
	LimeLightActivateBlueScaleCommand() : InstantCommand("LimeLightActivateBlueScaleCommand"){

	}
	void Initialize(){
	CommandBase :: m_limeLight->ActivateBlueScale();
	}

};

#endif  // LimeLightActivateBlueScale_H
