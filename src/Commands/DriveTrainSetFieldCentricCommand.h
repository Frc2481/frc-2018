#ifndef SetFieldCentricCommand_H
#define SetFieldCentricCommand_H

#include "../CommandBase.h"

class SetFieldCentricCommand : public CommandBase {
private:
	bool m_state;
public:
	SetFieldCentricCommand(bool state) : CommandBase("SetFieldCentricCommand"){
		m_state = state;
	}
	void Initialize(){
		CommandBase::m_driveTrain->ZeroGyro();
		CommandBase::m_driveTrain->SetFieldCentric(m_state);
	}
	void Execute(){}
	bool IsFinished(){
		return true;
	}
	void End(){}
	void Interrupted(){}
};

#endif  // SetFieldCentricCommand_H
