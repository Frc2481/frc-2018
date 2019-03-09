#ifndef IntakeRollerLoadCommand_H
#define IntakeRollerLoadCommand_H

#include "CommandBase.h"

class IntakeRollerLoadCommand : public frc::InstantCommand {
private:
	double m_speed;
public:
	IntakeRollerLoadCommand(double speed) : InstantCommand("IntakeRollerLoadCommand"){
		m_speed = speed;
	}

	void Initialize(){
		CommandBase::m_intake->RollerLoad(m_speed);
	}



};

#endif  // IntakeRollerLoadCommand_H
