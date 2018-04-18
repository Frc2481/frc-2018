/*
 * Intake.h
 *
 *  Created on: Jan 15, 2018
 *      Author: Team2481
 */

#ifndef SRC_SUBSYSTEMS_INTAKE_H_
#define SRC_SUBSYSTEMS_INTAKE_H_

#include "ctre/Phoenix.h"
#include "WPILib.h"



class Intake : public Subsystem{
private:
	TalonSRX* m_rollerMotorLeft;
	TalonSRX* m_rollerMotorRight;
	Solenoid* m_clampSolenoid;

public:
	Intake();
	virtual ~Intake();

	bool HasCube();
	bool HasCubeSecure();

	bool IsLeftRollerOn();
	bool IsRightRollerOn();
	void RollerLoad(double speed);
	void RollerUnload(double speed);
	void RollerOff();

	bool IsClamped();
	void OpenClamp();
	void CloseClamp();

	void Periodic();
};

#endif /* SRC_SUBSYSTEMS_INTAKE_H_ */
