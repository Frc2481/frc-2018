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
#include "RobotParameters.h"
#include "ArmRetractWhenExtendedCommand.h"

template <int EXT, int PIVOT_ANGLE>
class ArmBaseCommand : public CommandBase{
private:
	bool m_pivotOnTarget;
	bool m_extensionOnTarget;
public:
	ArmBaseCommand(std::string name) : CommandBase(name) {
		Requires(m_arm.get());
	}
	virtual ~ArmBaseCommand() {}
	void Initialize() {
		m_arm->SetDesiredExtension(EXT);
		m_arm->SetPivotAngle(Rotation2D::fromDegrees(PIVOT_ANGLE));
		m_pivotOnTarget = false;
		m_extensionOnTarget = false;
	}
	void Execute() {
		m_arm->SetExtensionPosition(m_arm->GetAllowedExtensionPos());
		SmartDashboard::PutNumber("get allowed extension pos", m_arm->GetAllowedExtensionPos());
	}
	bool IsFinished() {
		if(CommandBase::m_arm->IsPivotOnTarget()) {
			m_pivotOnTarget = true;
		}
		if(CommandBase::m_arm->IsExtensionOnTarget()) {
			m_extensionOnTarget = true;
		}
		return m_pivotOnTarget && m_extensionOnTarget;
	}

	void End() {
		SmartDashboard::PutNumber("arm movement time", TimeSinceInitialized());
	}

	void Interrupted() {
		End();
	}
};

template <int EXT, int PIVOT_ANGLE>
class ArmBaseCommandGroup : public CommandGroup {
public:
	static const int k_extensionPos = EXT;
	static const int k_pivotAngle = PIVOT_ANGLE;

	ArmBaseCommandGroup(std::string name) : CommandGroup(name) {
		AddSequential(new PrintCommand("Arm Start Base Command Group"));
		AddSequential(new ArmRetractWhenExtendedCommand(PIVOT_ANGLE), 1.5);
		AddSequential(new PrintCommand("Arm Mid Base Command Group"));
		AddSequential(new ArmBaseCommand<EXT, PIVOT_ANGLE>(""), 1.5);
		AddSequential(new PrintCommand("Arm End Base Command Group"));
	}
};

typedef ArmBaseCommandGroup<2, 112> ArmToExchangeFront;
typedef ArmBaseCommandGroup<2, -112> ArmToExchangeBack;

typedef ArmBaseCommandGroup<4, 122> ArmToIntakeFront;
typedef ArmBaseCommandGroup<4, -121> ArmToIntakeBack;

typedef ArmBaseCommandGroup<1, 104> ArmToIntake2Front;
typedef ArmBaseCommandGroup<1, -101> ArmToIntake2Back;

typedef ArmBaseCommandGroup<0, 85> ArmToIntake3Front;
typedef ArmBaseCommandGroup<0, -83> ArmToIntake3Back;

typedef ArmBaseCommandGroup<0, 71> ArmToSwitchFront;
typedef ArmBaseCommandGroup<0, -66> ArmToSwitchBack;

typedef ArmBaseCommandGroup<0, 66> ArmToSwitch2Front;
typedef ArmBaseCommandGroup<0, -66> ArmToSwitch2Back;

typedef ArmBaseCommandGroup<9, 35> ArmToLowScaleFront; //15, 44
typedef ArmBaseCommandGroup<9, -35> ArmToLowScaleBack; // 11, -45

typedef ArmBaseCommandGroup<8, 28> ArmToLowScale2Front;
typedef ArmBaseCommandGroup<8, -28> ArmToLowScale2Back;

typedef ArmBaseCommandGroup<24, 25> ArmToMidScaleFront;
typedef ArmBaseCommandGroup<24, -25> ArmToMidScaleBack;

typedef ArmBaseCommandGroup<30, 23> ArmToMidScale2Front;
typedef ArmBaseCommandGroup<30, -23> ArmToMidScale2Back;

typedef ArmBaseCommandGroup<36, 17> ArmToHighScaleFront; //<36, 22>
typedef ArmBaseCommandGroup<36, -22> ArmToHighScaleBack; //<36, -22>

typedef ArmBaseCommandGroup<36, 16> ArmToHighScale2Front;
typedef ArmBaseCommandGroup<36, -16> ArmToHighScale2Back;


typedef ArmBaseCommandGroup<7, 125> ArmCubesToExchangeFront;
typedef ArmBaseCommandGroup<4, -123> ArmCubesToExchangeBack;

typedef ArmBaseCommandGroup<0, 0> ArmToStow;

typedef ArmBaseCommandGroup<0, 90> ArmTo90Front;
typedef ArmBaseCommandGroup<0, -90> ArmTo90Back;


#endif /* SRC_COMMANDS_ARMBASECOMMAND_H_ */
