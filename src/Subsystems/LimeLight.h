/*
 * LimeLight.h
 *
 *  Created on: Jan 18, 2018
 *      Author: Team2481
 */

#ifndef SRC_SUBSYSTEMS_LIMELIGHT_H_
#define SRC_SUBSYSTEMS_LIMELIGHT_H_


class LimeLight {
public:
	LimeLight();
	virtual ~LimeLight();
	float targetOffsetAngle_Horizontal();
	float targetOffsetAngle_Vertical();
	float targetArea();
	float targetSkew();

	void TurnOnLED();
	void TurnOffLED();
	void BlinkLight();

	void ActivateRedScale();
	void ActivateBlueScale();
	void PowerCubePipeline();
	void ActivateFencePipeline();
};
//std::shared_ptr<NetworkTable> table =   NetworkTable::GetTable("limelight");
//float targetOffsetAngle_Horizontal = table->GetNumber("tx");
//float targetOffsetAngle_Vertical = table->GetNumber("ty");
//float targetArea = table->GetNumber("ta");
//float targetSkew = table->GetNumber("ts");
#endif /* SRC_SUBSYSTEMS_LIMELIGHT_H_ */
