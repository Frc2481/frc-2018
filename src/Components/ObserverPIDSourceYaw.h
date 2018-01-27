/*
 * ObserverPIDSourceYaw.h
 *
 *  Created on: Jan 26, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMPONENTS_OBSERVERPIDSOURCEYAW_H_
#define SRC_COMPONENTS_OBSERVERPIDSOURCEYAW_H_

#include "WPILib.h"
#include "../Subsystems/Observer.h"

class ObserverPIDSourceYaw : public PIDSource {
public:
	ObserverPIDSourceYaw(Observer* src);
	virtual ~ObserverPIDSourceYaw();

	double PIDGet();

private:
	Observer* obj;
};

#endif /* SRC_COMPONENTS_OBSERVERPIDSOURCEYAW_H_ */
