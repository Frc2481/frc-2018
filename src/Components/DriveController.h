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
#include "../PIDController2481.h"

class DriveController {
public:
	DriveController(Observer* observerObj);
	virtual ~DriveController();

	void SetFieldTarget(RigidTransform2D fieldTarget);
	void SetRobotTarget(RigidTransform2D robotTarget);
	void EnableController();
	bool IsOnTarget();
	RigidTransform2D GetControllerError();
	RigidTransform2D GetDriveControlSignal();

private:

	PIDController2481* m_positionXController;
	PIDController2481* m_positionYController;
	PIDController2481* m_positionYawController;

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
