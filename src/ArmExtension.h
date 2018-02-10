/*
 * ArmExtension.h
 *
 *  Created on: Feb 1, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_ARMEXTENSION_H_
#define SRC_ARMEXTENSION_H_

#include <cmath>
#include <algorithm>
#include "utils/Rotation2D.h"
#include "RobotParameters.h"
#include "WPILib.h"

class ArmExtension {
public:
	ArmExtension();
	virtual ~ArmExtension();
	double MaxExtension(Rotation2D &pivotAngle);
	double MinExtension(Rotation2D &pivotAngle);
	double Constrain(double extension, Rotation2D pivotAngle);
};

#endif /* SRC_ARMEXTENSION_H_ */
