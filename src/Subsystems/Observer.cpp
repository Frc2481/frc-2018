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

	m_flLineSensor = new DigitalInput(1);
	m_frLineSensor = new DigitalInput(2);
	m_blLineSensor = new DigitalInput(3);
	m_brLineSensor = new DigitalInput(4);

	m_flLineSensorOffset = Translation2D(RobotParameters::k_lineDetectXOffsetFL, RobotParameters::k_lineDetectYOffsetFL);
	m_frLineSensorOffset = Translation2D(RobotParameters::k_lineDetectXOffsetFR, RobotParameters::k_lineDetectYOffsetFR);
	m_blLineSensorOffset = Translation2D(RobotParameters::k_lineDetectXOffsetBL, RobotParameters::k_lineDetectYOffsetBL);
	m_brLineSensorOffset = Translation2D(RobotParameters::k_lineDetectXOffsetBR, RobotParameters::k_lineDetectYOffsetBR);

	m_xCorrection = 0;
	m_xCorrectionCount = 0;
	m_yCorrection = 0;
	m_yCorrectionCount = 0;
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

	m_isFlLineDetected = m_flLineSensor->Get();
	m_isFrLineDetected = m_frLineSensor->Get();
	m_isBlLineDetected = m_blLineSensor->Get();
	m_isBrLineDetected = m_brLineSensor->Get();

	LineCrossed lineState = NO_LINE;
	if(m_isFlLineDetected || m_isFrLineDetected || m_isBlLineDetected || m_isBrLineDetected) {
		lineState = LinePosCorrection();

		switch(lineState) {
			case NULL_LEFT_CLOSE: {
				double lineSensorYOffset = m_flLineSensorOffset.rotateBy(robotPos.getRotation()).getY();
				ResetPoseY(RobotParameters::k_leftNullZoneClose - lineSensorYOffset);
				break;
			}
			case NULL_RIGHT_CLOSE: {
				double lineSensorYOffset = m_frLineSensorOffset.rotateBy(robotPos.getRotation()).getY();
				ResetPoseY(RobotParameters::k_rightNullZoneClose - lineSensorYOffset);
				break;
			}
			case PLATFORM_LEFT_CLOSE: {
				double lineSensorXOffset = m_brLineSensorOffset.rotateBy(robotPos.getRotation()).getX();
				ResetPoseX(RobotParameters::k_leftPlatformClose - lineSensorXOffset);
				break;
			}
			case PLATFORM_RIGHT_CLOSE: {
				double lineSensorXOffset = m_blLineSensorOffset.rotateBy(robotPos.getRotation()).getX();
				ResetPoseX(RobotParameters::k_rightPlatformClose - lineSensorXOffset);
				break;
			}
			case NULL_LEFT_FAR: {
				double lineSensorYOffset = m_flLineSensorOffset.rotateBy(robotPos.getRotation()).getY();
				ResetPoseY(RobotParameters::k_leftNullZoneFar - lineSensorYOffset);
				break;
			}
			case NULL_RIGHT_FAR: {
				double lineSensorYOffset = m_frLineSensorOffset.rotateBy(robotPos.getRotation()).getY();
				ResetPoseY(RobotParameters::k_rightNullZoneFar - lineSensorYOffset);
				break;
			}
			case PLATFORM_LEFT_FAR: {
				double lineSensorXOffset = m_brLineSensorOffset.rotateBy(robotPos.getRotation()).getX();
				ResetPoseX(RobotParameters::k_leftPlatformFar - lineSensorXOffset);
				break;
			}
			case PLATFORM_RIGHT_FAR: {
				double lineSensorXOffset = m_blLineSensorOffset.rotateBy(robotPos.getRotation()).getX();
				ResetPoseX(RobotParameters::k_rightPlatformFar - lineSensorXOffset);
				break;
			}
			default:
				break;
		}
	}

//	if(m_xCorrectionCount > 0) {
//		m_xCorrectionCount--;
//		RigidTransform2D &robotPos = m_robotPos.rbegin()->second;
//		robotPos.getTranslation().setX(robotPos.getTranslation().getX() - m_xCorrection / 50.0);
//	}

//	if(m_yCorrectionCount > 0) {
//		m_yCorrectionCount--;
//		RigidTransform2D &robotPos = m_robotPos.rbegin()->second;
//		robotPos.getTranslation().setY(robotPos.getTranslation().getY() - m_yCorrection / 24.0);
//	}

	SmartDashboard::PutBoolean("FL Line Sensor", m_isFlLineDetected);
	SmartDashboard::PutBoolean("FR Line Sensor", m_isFrLineDetected);
	SmartDashboard::PutBoolean("BL Line Sensor", m_isBlLineDetected);
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
//	RigidTransform2D &robotPos = m_robotPos.rbegin()->second;
//	m_xCorrection = robotPos.getTranslation().getX() - x;
//	m_xCorrectionCount = 50;

	RigidTransform2D robotPos = GetLastRobotPose();
	printf("reset x prev %f new %f\n", robotPos.getTranslation().getX(), x);
	robotPos.getTranslation().setX(x);
	SetRobotPos(robotPos, RobotController::GetFPGATime());
}

void Observer::ResetPoseY(double y) {
//	RigidTransform2D &robotPos = m_robotPos.rbegin()->second;
//	m_yCorrection = robotPos.getTranslation().getY() - y;
//	m_yCorrectionCount = 24;

	RigidTransform2D robotPos = GetLastRobotPose();
	printf("reset y prev %f new %f\n", robotPos.getTranslation().getY(), y);
	robotPos.getTranslation().setY(y);
	SetRobotPos(robotPos, RobotController::GetFPGATime());
}

RigidTransform2D Observer::GetLastRobotPose() {
	return m_robotPos.rbegin()->second;
}

LineCrossed Observer::LinePosCorrection() {
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


	if(sensorYFL > (RobotParameters::k_leftNullZoneClose - RobotParameters::k_lineDetectZone) &&
			sensorYFL < (RobotParameters::k_leftNullZoneClose + RobotParameters::k_lineDetectZone) &&
			sensorXFL < 162 &&
			m_isFlLineDetected) {
		return NULL_LEFT_CLOSE;
	}
	else if(sensorYFR > (RobotParameters::k_rightNullZoneClose - RobotParameters::k_lineDetectZone) &&
			sensorYFR < (RobotParameters::k_rightNullZoneClose + RobotParameters::k_lineDetectZone) &&
			sensorXFR > 162 &&
			m_isFrLineDetected) {
		return NULL_RIGHT_CLOSE;
	}
	else if(sensorXBR > (RobotParameters::k_leftPlatformClose - RobotParameters::k_lineDetectZone) &&
			sensorXBR < (RobotParameters::k_leftPlatformClose + RobotParameters::k_lineDetectZone) &&
			sensorYBR > 200 &&
			sensorYBR < 265 &&
			m_isBrLineDetected) {
		return PLATFORM_LEFT_CLOSE;
	}
	else if(sensorXBL > (RobotParameters::k_rightPlatformClose - RobotParameters::k_lineDetectZone) &&
			sensorXBL < (RobotParameters::k_rightPlatformClose + RobotParameters::k_lineDetectZone) &&
			sensorYBL > 200 &&
			sensorYBL < 265 &&
			m_isBlLineDetected) {
		return PLATFORM_RIGHT_CLOSE;
	}
	else if(sensorYFL > (RobotParameters::k_leftNullZoneFar - RobotParameters::k_lineDetectZone) &&
			sensorYFL < (RobotParameters::k_leftNullZoneFar + RobotParameters::k_lineDetectZone) &&
			sensorXFL < 162 &&
			m_isFlLineDetected) {
		return NULL_LEFT_FAR;
	}
	else if(sensorYFR > (RobotParameters::k_rightNullZoneFar - RobotParameters::k_lineDetectZone) &&
			sensorYFR < (RobotParameters::k_rightNullZoneFar + RobotParameters::k_lineDetectZone) &&
			sensorXFR > 162 &&
			m_isFrLineDetected) {
		return NULL_RIGHT_FAR;
	}
	else if(sensorXBR > (RobotParameters::k_leftPlatformFar - RobotParameters::k_lineDetectZone) &&
			sensorXBR < (RobotParameters::k_leftPlatformFar + RobotParameters::k_lineDetectZone) &&
			sensorYBR > 200 &&
			sensorYBR < 265 &&
			m_isBrLineDetected) {
		return PLATFORM_LEFT_FAR;
	}
	else if(sensorXBL > (RobotParameters::k_rightPlatformFar - RobotParameters::k_lineDetectZone) &&
			sensorXBL < (RobotParameters::k_rightPlatformFar + RobotParameters::k_lineDetectZone) &&
			sensorYBL > 200 &&
			sensorYBL < 265 &&
			m_isBlLineDetected) {
		return PLATFORM_RIGHT_FAR;
	}
	else {
		return NO_LINE;
	}
}
