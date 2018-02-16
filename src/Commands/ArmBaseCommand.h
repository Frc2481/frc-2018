/*
 * ArmBaseCommand.h
 *
 *  Created on: Feb 5, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMBASECOMMAND_H_
#define SRC_COMMANDS_ARMBASECOMMAND_H_

#include "CommandBase.h"
#include "Subsystems/Arm.h"
#include "ArmExtension.h"
#include "Commands/CommandGroup.h"
#include "Commands/ArmRetractWhenExtendedCommand.h"

template <int EXT, int PIVOT_ANGLE>
class ArmBaseCommand : public CommandBase{
private:
	int m_counter;
	int m_debounce;
public:
	ArmBaseCommand(std::string name) : CommandBase(name) {}
	virtual ~ArmBaseCommand() {}
	void Initialize() {
		m_arm->SetDesiredExtension(EXT);
		m_arm->SetPivotAngle(Rotation2D::fromDegrees(PIVOT_ANGLE));
		m_counter = 0;
		m_debounce = 0;
	}
	void Execute() {
		m_arm->SetExtensionPostion(m_arm->GetAllowedExtensionPos());
		SmartDashboard::PutNumber("get allowed extension pos", m_arm->GetAllowedExtensionPos());
		m_counter++;
		SmartDashboard::PutNumber("arm base command counter", m_counter);
		while((m_arm->GetPivotAngle().getDegrees() > 90 || m_arm->GetPivotAngle().getDegrees() < -90) &&
				fabs(fabs(m_arm->GetDesiredPivotAngle().getDegrees()) - fabs(m_arm->GetPivotAngle().getDegrees())) > 15) {
			m_arm->SetExtensionPostion(m_arm->GetExtensionPosition());
		}
	}
	bool IsFinished() {
		if(CommandBase::m_arm->IsPivotOnTarget()) {
			m_debounce++;
		}
		else {
			m_debounce = 0;
		}
		return m_debounce > 5;
	}
	void End() {}
	void Interrupted() {
		End();
	}
};

template <int EXT, int PIVOT_ANGLE>
class ArmBaseCommandGroup : public CommandGroup {
public:
	ArmBaseCommandGroup(std::string name) : CommandGroup(name) {
		AddSequential(new ArmRetractWhenExtendedCommand());
		AddSequential(new ArmBaseCommand<EXT, PIVOT_ANGLE>(""));
	}
};

typedef ArmBaseCommandGroup<4, 120> ArmToIntakeFront;
typedef ArmBaseCommandGroup<4, -119> ArmToIntakeBack;

typedef ArmBaseCommandGroup<1, 104> ArmToIntake2Front;
typedef ArmBaseCommandGroup<1, -101> ArmToIntake2Back;

typedef ArmBaseCommandGroup<0, 83> ArmToIntake3Front;
typedef ArmBaseCommandGroup<0, -83> ArmToIntake3Back;

//typedef ArmBaseCommandGroup<0, 91> ArmToSwitchFront;
//typedef ArmBaseCommandGroup<0, -86> ArmToSwitchBack;

typedef ArmBaseCommandGroup<8, 39> ArmToLowScaleFront; //15, 44
typedef ArmBaseCommandGroup<8, -39> ArmToLowScaleBack; // 11, -45

typedef ArmBaseCommandGroup<8, 28> ArmToLowScale2Front;
typedef ArmBaseCommandGroup<8, -28> ArmToLowScale2Back;

typedef ArmBaseCommandGroup<24, 25> ArmToMidScaleFront;
typedef ArmBaseCommandGroup<24, -25> ArmToMidScaleBack;

typedef ArmBaseCommandGroup<30, 23> ArmToMidScale2Front;
typedef ArmBaseCommandGroup<30, -23> ArmToMidScale2Back;

typedef ArmBaseCommandGroup<36, 22> ArmToHighScaleFront; // 35, 15
typedef ArmBaseCommandGroup<36, -22> ArmToHighScaleBack; //26, -20

typedef ArmBaseCommandGroup<36, 16> ArmToHighScale2Front;
typedef ArmBaseCommandGroup<36, -16> ArmToHighScale2Back;

typedef ArmBaseCommandGroup<7, 125> ArmCubesToExchangeFront;
typedef ArmBaseCommandGroup<4, -123> ArmCubesToExchangeBack;

typedef ArmBaseCommandGroup<0, 0> ArmToStow;

typedef ArmBaseCommandGroup<0, 90> ArmTo90Front;
typedef ArmBaseCommandGroup<0, -90> ArmTo90Back;


#endif /* SRC_COMMANDS_ARMBASECOMMAND_H_ */
