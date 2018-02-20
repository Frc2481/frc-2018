/*
 * ArmExtentionMotionScaling.h
 *
 *  Created on: Feb 17, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_COMMANDS_ARMEXTENTIONMOTIONSCALING_H_
#define SRC_COMMANDS_ARMEXTENTIONMOTIONSCALING_H_

#include "CommandBase.h"
#include "Subsystems/Arm.h"

class ArmExtentionMotionScaling : public InstantCommand{
private:
	double m_scale;
public:
	ArmExtentionMotionScaling(double scale) : InstantCommand("ArmExtensionMotionScaling") {
		m_scale = scale;
	}
	virtual ~ArmExtentionMotionScaling(){}

	void Initialize(){
		CommandBase::m_arm->SetExtentionMotionScaling(m_scale);
	}
};

#endif /* SRC_COMMANDS_ARMEXTENTIONMOTIONSCALING_H_ */
