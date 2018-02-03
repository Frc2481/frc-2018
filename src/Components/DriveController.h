/*
 * DriveController.h
 *
 *  Created on: Jan 25, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMPONENTS_DRIVECONTROLLER_H_
#define SRC_COMPONENTS_DRIVECONTROLLER_H_

#include "../utils/RigidTransform2D.h"
#include "../Subsystems/Observer.h"
#include "ObserverPIDSourceX.h"
#include "ObserverPIDSourceY.h"
#include "ObserverPIDSourceYaw.h"
#include "DriveControllerOutput.h"
#include "WPILib.h"
#include "RobotParameters.h"

class DriveController {
public:
	DriveController(Observer* observerObj);
	virtual ~DriveController();

	void SetFieldTarget(RigidTransform2D fieldTarget, RigidTransform2D absTolerance);
	void SetRobotTarget(RigidTransform2D robotTarget, RigidTransform2D absTolerance);
	void SetPIDGains(double kpPos, double kiPos, double kdPos, double kpYaw, double kiYaw, double kdYaw);
	void EnableController();
	bool IsOnTarget();

	RigidTransform2D GetDriveControlSignal();

//private:

	PIDController* m_positionXController;
	PIDController* m_positionYController;
	PIDController* m_positionYawController;

	ObserverPIDSourceX* m_positionXControlSource;
	ObserverPIDSourceY* m_positionYControlSource;
	ObserverPIDSourceYaw* m_positionYawControlSource;

	DriveControllerOutput* m_positionXControlSignal;
	DriveControllerOutput* m_positionYControlSignal;
	DriveControllerOutput* m_positionYawControlSignal;

	Observer* m_observer;

	void ResetController();
};

#endif /* SRC_COMPONENTS_DRIVETRAINDRIVECONTROLLER_H_ */
