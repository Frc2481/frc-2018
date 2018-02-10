/*
 * ObserverPIDSourceY.h
 *
 *  Created on: Jan 26, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMPONENTS_OBSERVERPIDSOURCEY_H_
#define SRC_COMPONENTS_OBSERVERPIDSOURCEY_H_

#include "WPILib.h"
#include "../Subsystems/Observer.h"

class ObserverPIDSourceY : public PIDSource {
public:
	ObserverPIDSourceY(Observer* src);
	virtual ~ObserverPIDSourceY();

	double PIDGet();

private:
	Observer* obj;
};

#endif /* SRC_COMPONENTS_OBSERVERPIDSOURCEY_H_ */
