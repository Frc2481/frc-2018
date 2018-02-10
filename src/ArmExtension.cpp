/*
 * ArmExtension.cpp
 *
 *  Created on: Feb 1, 2018
 *      Author: FIRSTMentor
 */

#include "ArmExtension.h"

ArmExtension::ArmExtension() {
	// TODO Auto-generated constructor stub

}

ArmExtension::~ArmExtension() {
	// TODO Auto-generated destructor stub
}

double ArmExtension::MaxExtension(Rotation2D &pivotAngle) {
	double extension = 0;
	double angle = pivotAngle.getDegrees();
	double angleSin = fabs(pivotAngle.getSin());
	double angleTan = Rotation2D::fromDegrees(fabs(angle)).getTan();
//	if (angle < RobotParameters::k_rightFrameConstrained && angle > RobotParameters::k_leftFrameConstrained) {
//		extension = 42;
//	}
//	//top quadrant (Zone 3)
	if(angle > RobotParameters::k_upperLeftBound && angle < RobotParameters::k_upperRightBound) {
		extension = 42; //change depending on where we actually want this
	}

	//upper right to 90 degrees right (Zone 4)
	else if(angle >= RobotParameters::k_upperRightBound && angle < RobotParameters::k_right90) {
		extension = (RobotParameters::k_farthestPointLimit / angleSin) + (RobotParameters::k_pivotToMidpointPOB / angleTan) -
				RobotParameters::k_minRobotExtend;
	}

	//90 degrees right to floor (Zone 5)
	else if(angle >= RobotParameters::k_right90 && angle <= RobotParameters::k_lowerRightBound) {
		extension = (RobotParameters::k_farthestPointLimit / angleSin) - (RobotParameters::k_pivotToMidpointPOB / angleTan) -
				RobotParameters::k_minRobotExtend - (RobotParameters::k_gripperThickness / Rotation2D::fromDegrees(180 - fabs(angle)).getTan());
	}

	//upper left to 90 degrees left (Zone 2
	else if(angle < RobotParameters::k_upperLeftBound && angle > RobotParameters::k_left90) {
		extension = (RobotParameters::k_farthestPointLimit / angleSin) - (RobotParameters::k_pivotToMidpointPOT / angleTan) -
				RobotParameters::k_minRobotExtend;
	}

	// 90 degrees left to floor (Zone 1)
	else if(angle <= RobotParameters::k_left90 && angle >= RobotParameters::k_lowerLeftBound) {
	    extension = (RobotParameters::k_farthestPointLimit / angleSin) + (RobotParameters::k_pivotToMidpointPOT / angleTan) -
				RobotParameters::k_minRobotExtend - (RobotParameters::k_gripperThickness / Rotation2D::fromDegrees(180 - fabs(angle)).getTan());
	}

	return extension;
}

double ArmExtension::MinExtension(Rotation2D &pivotAngle) {
	double extension = 0;
	double angle = pivotAngle.getDegrees();
	//fix later, temp testing
//	if(angle >= RobotParameters::k_leftFrameConstrained && angle <= RobotParameters::k_rightFrameConstrained) {
//		extension = 5.5; //giving tolerance, originally 4.75
//	}

	extension = 7.0;

	return extension;
}

double ArmExtension::Constrain(double extension, Rotation2D pivotAngle) {
	SmartDashboard::PutNumber("extension constrain1", extension);
	extension = std::max(MinExtension(pivotAngle), extension);
	extension = std::min(MaxExtension(pivotAngle), extension);
	extension = std::max(extension, 7.0); //temp testing
	SmartDashboard::PutNumber("extension constrain2", extension);
	SmartDashboard::PutNumber("min extension", MinExtension(pivotAngle));
	SmartDashboard::PutNumber("max extension", MaxExtension(pivotAngle));
	SmartDashboard::PutNumber("constrained pivot angle", pivotAngle.getDegrees());
	return extension;
}
