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
#include "../PVAController.h"
#include "utils/PathLoader.h"

class DriveController {
public:
	DriveController(Observer* observerObj);
	virtual ~DriveController();

	void SetFieldTarget(PathPoint2D &fieldTarget);
	void SetRobotTarget(PathPoint2D &robotTarget);
	RigidTransform2D GetControllerError();
	RigidTransform2D GetDriveControlSignal();
	void SetPositionGains(double kp, double kv, double kap, double kan, double kd);
	void SetYawGains(double kp, double kv, double kap, double kan, double kd);

	Observer* GetObserver();

private:

	PVAController* m_positionXController;
	PVAController* m_positionYController;
	PVAController* m_positionYawController;

	Observer* m_observer;

	void ResetController();
};

#endif /* SRC_COMPONENTS_DRIVETRAINDRIVECONTROLLER_H_ */
