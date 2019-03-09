/*
 * ObserverPIDSourceYaw.h
 *
 *  Created on: Jan 26, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMPONENTS_OBSERVERPIDSOURCEYAW_H_
#define SRC_COMPONENTS_OBSERVERPIDSOURCEYAW_H_

#include "Subsystems/Observer.h"
#include "frc/WPIlib.h"

class ObserverPIDSourceYaw : public frc::PIDSource {
public:
	ObserverPIDSourceYaw(Observer* src);
	virtual ~ObserverPIDSourceYaw();

	double PIDGet();

private:
	Observer* obj;
};

#endif /* SRC_COMPONENTS_OBSERVERPIDSOURCEYAW_H_ */
