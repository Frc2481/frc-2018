#ifndef IntakeRollerUnloadCommand_H
#define IntakeRollerUnloadCommand_H

#include "CommandBase.h"

class IntakeRollerUnloadCommand : public frc::InstantCommand {
private:
	double m_speed;
public:
	IntakeRollerUnloadCommand(double speed) : InstantCommand("IntakeRollerUnloadCommand"){
		m_speed = speed;
	}
	void Initialize(){
		CommandBase::m_intake->RollerUnload(m_speed);
	}

};

#endif  // IntakeRollerUnloadCommand_H

