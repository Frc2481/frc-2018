#ifndef IntakeCubeHasCommand_H
#define IntakeCubeHasCommand_H

#include "CommandBase.h"

class IntakeHasCubeCommand : public CommandBase {
private:
	int m_debounceCounter;
public:
	IntakeHasCubeCommand() : CommandBase("IntakeHasCubeCommand"){
		m_debounceCounter = 0;
//		SetTimeout(3);
	}

	void Initialize() {
		m_debounceCounter = 0;
	}

	void Execute() {}

	bool IsFinished(){
//		if(m_intake->HasCube()) {
//			m_debounceCounter++;
//		}
//		else {
//			m_debounceCounter = 0;
//		}
//		return m_debounceCounter > 2;
		return m_intake->HasCube(); // || IsTimedOut();
	}
};

#endif  // IntakeCubeHasCommand_H
