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
#include "DriveTrain2017.h"

class Intake {
private:
	TalonSRX* m_rollerMotor;
	Solenoid* m_clampSolenoid;
public:
	Intake();
	virtual ~Intake();

	bool HasCube();

	bool IsRollerOn();
	void RollerLoad();
	void RollerUnload();
	void RollerOff();

	bool IsClamped();
	void OpenClamp();
	void CloseClamp();
};

#endif /* SRC_SUBSYSTEMS_INTAKE_H_ */
