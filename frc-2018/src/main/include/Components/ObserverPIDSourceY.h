/*
 * ObserverPIDSourceY.h
 *
 *  Created on: Jan 26, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMPONENTS_OBSERVERPIDSOURCEY_H_
#define SRC_COMPONENTS_OBSERVERPIDSOURCEY_H_

#include "Subsystems/Observer.h"
#include "frc/WPILib.h"

class ObserverPIDSourceY : public frc::PIDSource {
public:
	ObserverPIDSourceY(Observer* src);
	virtual ~ObserverPIDSourceY();

	double PIDGet();

private:
	Observer* obj;
};

#endif /* SRC_COMPONENTS_OBSERVERPIDSOURCEY_H_ */
