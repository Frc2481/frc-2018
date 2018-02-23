/*
 * LimeLight.h
 *
 *  Created on: Jan 18, 2018
 *      Author: Team2481
 */

#ifndef SRC_SUBSYSTEMS_LIMELIGHT_H_
#define SRC_SUBSYSTEMS_LIMELIGHT_H_

#include "Commands/Subsystem.h"
#include "utils/RigidTransform2D.h"
#include "RobotParameters.h"
#include "cmath"
#include "algorithm"

class LimeLight : public Subsystem{
public:
	LimeLight();
	virtual ~LimeLight();
	double getPowerCubeTargetValid();
	double getPowerCubeTargetOffsetAngleHorizontal();
	double getPowerCubeTargetOffsetAngleVertical();
	double getPowerCubeTargetArea();
	double getPowerCubeTargetSkew();

	void SetPowerCubePose(RigidTransform2D cubePose);
	RigidTransform2D GetPowerCubePose();

	void TurnOnLED();
	void TurnOffLED();
	void BlinkLED();

	void ActivatePowerCubePipeline();
//	void ActivateRedScalePipeline();
//	void ActivateBlueScalePipeline();
//	void ActivateFencePipeline();

	void CalculatePowerCubePose();

	virtual void Periodic();

private:
	RigidTransform2D m_powerCubePose;
};

#endif /* SRC_SUBSYSTEMS_LIMELIGHT_H_ */
