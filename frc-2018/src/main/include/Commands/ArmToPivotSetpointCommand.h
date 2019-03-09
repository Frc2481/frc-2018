/*
 * ArmToPivotSetpoint.h
 *
 *  Created on: Jan 29, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMTOPIVOTSETPOINTCOMMAND_H_
#define SRC_COMMANDS_ARMTOPIVOTSETPOINTCOMMAND_H_

#include "frc/WPILib.h"
#include "CommandBase.h"
#include "Subsystems/Arm.h"
#include "ArmExtension.h"

using namespace frc;

class ArmToPivotSetpointCommand : public CommandBase{
private:
	int m_setpoint;
	int m_midpoint;
	bool m_isForward;
	int m_counter;
//	int m_seventyFivePercent;
	double m_extension;
public:
	ArmToPivotSetpointCommand(const std::string name, int setpoint) : CommandBase(name) {
		m_setpoint = setpoint;
		m_midpoint = 0;
		m_isForward = true;
		m_counter = 0;
//		m_seventyFivePercent = 0;
		m_extension = 0;
	}
	void Initialize() {
		m_arm->SetPivotAccel(1000);
		double pivotAngle = m_arm->GetPivotAngle().getDegrees();
		m_arm->SetPivotAngle(Rotation2D::fromDegrees(m_setpoint));
		m_midpoint = (m_setpoint + pivotAngle) / 2.0;
//		m_seventyFivePercent = (m_setpoint + pivotAngle) * 0.75;
		m_isForward = m_setpoint > pivotAngle;
		SmartDashboard::PutNumber("midpoint", m_midpoint);
		SmartDashboard::PutNumber("forward", (int)m_isForward);
	}
	void Execute() {
//		bool pastMidpoint = (m_isForward && (m_midpoint < m_arm->GetPivotAngle())) ||
//							(!m_isForward && (m_midpoint >= m_arm->GetPivotAngle()));
//		bool pastSeventyFive = (m_isForward && (m_seventyFivePercent < m_arm->GetPivotAngle())) ||
//				(!m_isForward && (m_seventyFivePercent >= m_arm->GetPivotAngle()));
//
//		if(pastMidpoint) {
//			m_arm->SetPivotAccel(650);
//		}
//		else if(pastSeventyFive){
//			m_arm->SetPivotAccel(300);
//		}
//		else {
//			m_arm->SetPivotAccel(1000);
//		}

		bool pastMidpoint = (m_isForward && (m_midpoint < m_arm->GetPivotAngle().getDegrees())) ||
							(!m_isForward && (m_midpoint >= m_arm->GetPivotAngle().getDegrees()));
		if(pastMidpoint) {
			m_arm->SetPivotAccel(50);
		}
		else {
			m_arm->SetPivotAccel(750);
		}
//		SmartDashboard::PutNumber("pastMidpoint", (int)pastMidpoint);
		m_counter++;
		SmartDashboard::PutNumber("counter", m_counter);
	}
	bool IsFinished() {
		return m_arm->IsPivotOnTarget();
	}
	void End() {

	}
	void Interrupted() {
		End();
	}
};

class ArmPivotToCenterCommand : public ArmToPivotSetpointCommand {
public:
	ArmPivotToCenterCommand(): ArmToPivotSetpointCommand("ArmPivotToCenterCommand", 0) {}
};

class ArmPivotTo90Command : public ArmToPivotSetpointCommand {
public:
	ArmPivotTo90Command(): ArmToPivotSetpointCommand("ArmPivotTo90Command", 90) {}
};

class ArmPivotToNeg90Command : public ArmToPivotSetpointCommand {
public:
	ArmPivotToNeg90Command(): ArmToPivotSetpointCommand("ArmPivotToNeg90Command", -90) {}
};


#endif /* SRC_COMMANDS_ARMTOPIVOTSETPOINTCOMMAND_H_ */
