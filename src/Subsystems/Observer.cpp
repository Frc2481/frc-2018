/*
 * Observer.cpp
 *
 *  Created on: Jan 9, 2018
 *      Author: FIRSTMentor
 */

#include <Subsystems/Observer.h>
#include "RobotParameters.h"
#include <cmath>
#include <algorithm>
#include "WPILib.h"

Observer::Observer() : m_robotPos(5){
	m_isFlLineDetected = false;
	m_isFrLineDetected = false;
	m_isBlLineDetected = false;
	m_isBrLineDetected = false;

	m_flLineSensor = new DigitalInput(2);
	m_frLineSensor = new DigitalInput(4);
	m_blLineSensor = new DigitalInput(5);
	m_brLineSensor = new DigitalInput(1);
}

Observer::~Observer() {
	// TODO Auto-generated destructor stub
}

void Observer::UpdateRobotPoseObservation(Rotation2D& flAngle, RigidTransform2D::Delta& flVelocity, Rotation2D& frAngle,
	RigidTransform2D::Delta& frVelocity, Rotation2D& blAngle, RigidTransform2D::Delta& blVelocity,
	Rotation2D& brAngle, RigidTransform2D::Delta& brVelocity, double timestamp, Rotation2D& deltaGyroYaw) {
	RigidTransform2D::Delta deltaRobotPos = Kinematics::SwerveForwardKinematics(flAngle, flVelocity, frAngle, frVelocity, blAngle, blVelocity, brAngle, brVelocity, deltaGyroYaw);

	RigidTransform2D oldRobotPos = GetLastRobotPose();

	// Complementary filter to combine gyro and forward kinematic angle deltas together.
	Rotation2D finalAngleDelta = Rotation2D::fromRadians((deltaRobotPos.GetTheta() * kFwdKinematicsWeight) + (deltaGyroYaw.getRadians() * kGyroWeight));
	Rotation2D newRobotAngle = oldRobotPos.getRotation().rotateBy(finalAngleDelta);

	Translation2D newRobotTranslation = oldRobotPos.getTranslation().translateBy(Translation2D(deltaRobotPos.GetX(), deltaRobotPos.GetY()).rotateBy(newRobotAngle));

	RigidTransform2D robotPos(newRobotTranslation, newRobotAngle);
	SetRobotPos(robotPos, timestamp);

//	SmartDashboard::PutNumber("delta robot angle gyro", deltaGyroYaw.getDegrees());
//	SmartDashboard::PutNumber("delta robot angle kinematics", deltaRobotPos.GetTheta() * 180 / M_PI);
//	SmartDashboard::PutNumber("delta robot x kinematics", deltaRobotPos.GetX());
//	SmartDashboard::PutNumber("delta robot y kinematics", deltaRobotPos.GetY());

	bool m_isLineDetectedFL;
	bool m_isLineDetectedFR;
	bool m_isLineDetectedBL;
	bool m_isLineDetectedBR;

	m_isFlLineDetected = !m_flLineSensor->Get();
	m_isFrLineDetected = !m_frLineSensor->Get();
	m_isBlLineDetected = !m_blLineSensor->Get();
	m_isBrLineDetected = !m_brLineSensor->Get();

	int lineState = NO_LINE;
	if(m_isFlLineDetected || m_isFrLineDetected || m_isBlLineDetected || m_isBrLineDetected) {
		lineState = LinePosCorrection();
		double lineX = 0;
		double lineY = 0;

		switch(lineState) {
			case NULL_LEFT:
				lineX = GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectXOffsetFL
							- GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectYOffsetFL;

				lineY = GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectXOffsetFL
							+ GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectYOffsetFL;

				ResetPoseY(288 - lineY);
				break;
			case NULL_RIGHT:
				lineX = GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectXOffsetFR
							- GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectYOffsetFR;

				lineY = GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectXOffsetFR
							+ GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectYOffsetFR;

				ResetPoseY(288 - lineY);
				break;
			case PLATFORM_LEFT:
				lineX = GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectXOffsetBR
							- GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectYOffsetBR;

				lineY = GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectXOffsetBR
							+ GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectYOffsetBR;

				ResetPoseX(95 - lineX);
				break;
			case PLATFORM_RIGHT:
				lineX = GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectXOffsetBL
							- GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectYOffsetBL;

				lineY = GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectXOffsetBL
							+ GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectYOffsetBL;
				ResetPoseX(229 - lineX);
				break;
			default:
				break;
		}
	}

	SmartDashboard::PutBoolean("FL Line Sensor", m_isFlLineDetected);
	SmartDashboard::PutBoolean("BR Line Sensor", m_isBrLineDetected);
	SmartDashboard::PutNumber("Line State", lineState);
}

RigidTransform2D Observer::GetRobotPos(double timestamp) {
	return m_robotPos.getInterpolated(timestamp);
}

void Observer::SetRobotPos(RigidTransform2D robotPos, double timestamp) {
	m_robotPos.put(InterpolatingDouble(timestamp), robotPos);
}

void Observer::ResetPose(RigidTransform2D robotPose) {
	m_robotPos.clear();
	SetRobotPos(robotPose, RobotController::GetFPGATime() - 37);  // This number DOES NOT MATTER
	SetRobotPos(robotPose, RobotController::GetFPGATime());
}

void Observer::ResetPose() {
	m_robotPos.clear();
	Translation2D zeroTranslation = Translation2D(0, 0);
	Rotation2D zeroRotation = Rotation2D(1, 0, true);
	SetRobotPos(RigidTransform2D(zeroTranslation, zeroRotation), RobotController::GetFPGATime());
}

void Observer::ResetPoseX(double x) {
	RigidTransform2D robotPos = GetLastRobotPose();
	robotPos.getTranslation().setX(x);
	SetRobotPos(robotPos, RobotController::GetFPGATime());
}

void Observer::ResetPoseY(double y) {
	RigidTransform2D robotPos = GetLastRobotPose();
	robotPos.getTranslation().setY(y);
	SetRobotPos(robotPos, RobotController::GetFPGATime());
}

RigidTransform2D Observer::GetLastRobotPose() {
	return m_robotPos.rbegin()->second;
}

int Observer::LinePosCorrection() {
	//FL Sensor
	double sensorXFL = GetLastRobotPose().getTranslation().getX()
			+ GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectXOffsetFL
			- GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectYOffsetFL;

	double sensorYFL = GetLastRobotPose().getTranslation().getY()
			+ GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectXOffsetFL
			+ GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectYOffsetFL;

	//FR Sensor
	double sensorXFR = GetLastRobotPose().getTranslation().getX()
			+ GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectXOffsetFR
			- GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectYOffsetFR;

	double sensorYFR = GetLastRobotPose().getTranslation().getY()
			+ GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectXOffsetFR
			+ GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectYOffsetFR;

	//BL Sensor
	double sensorXBL = GetLastRobotPose().getTranslation().getX()
			+ GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectXOffsetBL
			- GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectYOffsetBL;

	double sensorYBL = GetLastRobotPose().getTranslation().getY()
			+ GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectXOffsetBL
			+ GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectYOffsetBL;

	//BR Sensor
	double sensorXBR = GetLastRobotPose().getTranslation().getX()
			+ GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectXOffsetBR
			- GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectYOffsetBR;

	double sensorYBR = GetLastRobotPose().getTranslation().getY()
			+ GetLastRobotPose().getRotation().getSin() * RobotParameters::k_lineDetectXOffsetBR
			+ GetLastRobotPose().getRotation().getCos() * RobotParameters::k_lineDetectYOffsetBR;

	printf("SensorXBR = %f\n", sensorXBR);
	printf("SensorYBR = %f\n", sensorYBR);

	if(sensorYFL > (288 - RobotParameters::k_lineDetectZone) &&
			sensorYFL < (288 + RobotParameters::k_lineDetectZone) &&
			sensorXFL < 162 &&
			m_isFlLineDetected) {
		return NULL_LEFT;
	}
	else if(sensorYFR > (288 - RobotParameters::k_lineDetectZone) &&
			sensorYFR < (288 + RobotParameters::k_lineDetectZone) &&
			sensorXFR > 162 &&
			m_isFrLineDetected) {
		return NULL_RIGHT;
	}
	else if(sensorXBR > (95 - RobotParameters::k_lineDetectZone) &&
			sensorXBR < (95 + RobotParameters::k_lineDetectZone) &&
			m_isBlLineDetected) {
		return PLATFORM_LEFT;
	}
	else if(sensorXBL > (229 - RobotParameters::k_lineDetectZone) &&
			sensorXBL < (229 + RobotParameters::k_lineDetectZone) &&
			m_isBrLineDetected) {
		return PLATFORM_RIGHT;
	}
	else {
		return NO_LINE;
	}
}
