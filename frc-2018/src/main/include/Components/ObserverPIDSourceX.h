/*
 * ObserverPIDSourceX.h
 *
 *  Created on: Jan 26, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMPONENTS_OBSERVERPIDSOURCEX_H_
#define SRC_COMPONENTS_OBSERVERPIDSOURCEX_H_

#include "Subsystems/Observer.h"
#include "frc/WPILib.h"

class ObserverPIDSourceX : public frc::PIDSource {
public:
	ObserverPIDSourceX(Observer* src);
	virtual ~ObserverPIDSourceX();

	double PIDGet();

private:
	Observer* obj;
};

#endif /* SRC_COMPONENTS_OBSERVERPIDSOURCEX_H_ */
